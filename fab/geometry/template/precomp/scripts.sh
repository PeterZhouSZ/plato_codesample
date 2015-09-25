#!/bin/sh -e

set -o nounset

# NOTE: This file contains all the scripts necessary to run distributed precomputation
# for a geometry template on AWS. Typically the process is semi-automatic with stages
# set here using flags follwed by manual checks that the intermediate results are sane.

# ONLY ABOUT HALF OF THE SCRIPT REMAINS FOR ILLUSTRATIVE PURPOSES.

# UTIL FUNCTIONS ---------------------------------------------------------------
function name_only() {
    yyy=`basename $1` bash -c 'echo "${yyy%.*}"'
}

# Obtains addresses of all currently running AWS hosts
function get_hosts() {
    echo "Getting hostnames"
    HOSTS=$LOCAL_RUN_DIR/hosts.txt
    ec2-describe-instances | grep INSTANCE | grep $CURRENT_AMI_ID | \
        grep "running" | awk '{print $4;}' > $HOSTS
    N_HOSTS=$(wc -l $HOSTS | awk '{printf "%d", $0;}')
    if [ "$N_HOSTS" -lt "$N_WORKERS" ]; then
        echo "ERROR! Requested ${N_WORKERS}, but only ${N_HOSTS} instances active: "
        cat $HOSTS
        echo "Please run: "
        echo "ec2-run-instances $CURRENT_AMI_ID -t m3.xlarge -k my-key-pair " \
            "-g shumash-security-group -n $((N_WORKERS - N_HOSTS))"
        exit 1
    fi
    echo "--> Ok: "
    cat $HOSTS
}

# e.g. $(host_number 5)
function host_number() {
    local NN=$1
    local LINE=$((NN + 1))
    sed -n "${LINE}{p;q;}" $HOSTS | awk '{printf "%s", $1}'
}


# SETTINGS ---------------------------------------------------------------------

# CS - chess set
# JB -jewelery box
# YY1(reg)
# YY2(circ)
# PS - platform sandal
# TC - toy car
# TLH - tea light holder
# HV - holey vase
DESIGN="JB"

# Machine image to use
CURRENT_AMI_ID="ami-dc5b69b4"

# Number of workers to distribute on
N_WORKERS=20

if [ "${DESIGN}" == "JB" ]; then
    # TESS VASE --> UNI & ADAPT DONE; GEOMETRY DOWNLOADED; CACHE OK; CUSTOMIZER PASTED
    TPL=/Users/shumash/Documents/Coding/rootrepo/data/fab/plato_examples/jbox/jbox.ascii_proto
    RUN_ID=geo_adaptive3
    UNI_N_SUBDIVISIONS=5
    ADAPT_DEPTH=4
fi

if [ "${DESIGN}" == "HV" ]; then
    # TESS VASE --> UNI & ADAPT DONE; GEOMETRY DOWNLOADED; CACHE OK; CUSTOMIZER PASTED
    TPL=/Users/shumash/Documents/Coding/rootrepo/data/fab/plato_examples/vase/tess_vase.ascii_proto
    RUN_ID=geo_adaptive7
    UNI_N_SUBDIVISIONS=5
    ADAPT_DEPTH=2
fi


if [ "${DESIGN}" == "YY1" ]; then
    # YIN YANG (REGULAR) --> UNI & ADAPT DONE; GEOMETRY DOWNLOADED; CACHE OK; CUSTOMIZER PASTED
    TPL=/Users/shumash/Documents/Coding/rootrepo/data/fab/plato_examples/yinyang/yinglass4.ascii_proto
    RUN_ID=geo_adaptive6
    UNI_N_SUBDIVISIONS=5
    ADAPT_DEPTH=4
fi


if [ "${DESIGN}" == "YY2" ]; then
    # YIN YANG (CIRCLE) --> UNI & ADAPT DONE; GEOMETRY DOWNLOADED; CACHE OK; CUSTOMIZER PASTED
    TPL=/Users/shumash/Documents/Coding/rootrepo/data/fab/plato_examples/yinyang/yinglass7.ascii_proto
    RUN_ID=geo_adaptive6
    UNI_N_SUBDIVISIONS=5
    ADAPT_DEPTH=4
fi

if [ "${DESIGN}" == "CS" ]; then
    # CHESS SET --> UNI & ADAPT DONE; GEOMETRY DOWNLOADED; CACHE OK; CUSTOMIZER PASTED
    TPL=/Users/shumash/Documents/Coding/rootrepo/data/fab/plato_examples/chess/chess_set.ascii_proto
    RUN_ID=geo_adaptive2
    UNI_N_SUBDIVISIONS=5
    ADAPT_DEPTH=3
    N_WORKERS=5
fi

if [ "${DESIGN}" == "TC" ]; then
    # CAR --> UNI & ADAPT DONE; GEOMETRY DOWNLOADED; CACHE OK; CUSTOMIZER PASTED
    TPL=/Users/shumash/Documents/Coding/rootrepo/data/fab/plato_examples/toycar/car3.ascii_proto
    RUN_ID=geo_adaptive1
    UNI_N_SUBDIVISIONS=5
    ADAPT_DEPTH=4
fi

if [ "${DESIGN}" == "TLH" ]; then
    TPL=/Users/shumash/Documents/Coding/rootrepo/data/fab/plato_examples/candle/candle2.ascii_proto
    RUN_ID=geo_adaptive2  #3_tentacles
    UNI_N_SUBDIVISIONS=4
    ADAPT_DEPTH=2
fi

if [ "${DESIGN}" == "PS" ]; then
    TPL=/Users/shumash/Documents/Coding/rootrepo/data/fab/plato_examples/heels/shoe5.ascii_proto
    RUN_ID=geo_adaptive11
    UNI_N_SUBDIVISIONS=5
    ADAPT_DEPTH=3
fi

# Whether to run locally or distribute
LOCAL=0

# Template name to determine directory structure
TPL_NAME=$(name_only $TPL)

# Stages to run
STAGES=:8:9:  #2:3: #4:  #:1:

# Base directories
S3_HOME_DIR=s3://mit-plato-test/precomp
S3_BINARIES=s3://mit-plato-test/bin
LOCAL_HOME=/Users/shumash
LOCAL_HOME_DIR=$LOCAL_HOME/Documents/3DModels/templates/precomp
AWS_HOME=/home/admin

LOCAL_RUN_DIR=$LOCAL_HOME_DIR/$TPL_NAME/$RUN_ID
LOCAL_TMP_DIR=$LOCAL_RUN_DIR/tmp
LOCAL_SCRIPT_DIR=$LOCAL_RUN_DIR/scripts
S3_RUN_DIR=$S3_HOME_DIR/$TPL_NAME/$RUN_ID
S3_TMP_DIR=$S3_RUN_DIR/tmp

USAGE="Runs PLATO precomputation on AMAZON

Template:      $TPL
Local run dir: $LOCAL_RUN_DIR
   S3 run dir: $S3_RUN_DIR

-s stages to run of the form :0:1:2:5: denotes the stages below to run:
   0 - build all necessary binaries & create scripts
   1 - update binaries
   2 - run uniform sampling (no computation)
   3 - copy files
   4 - run on amazon to get properties for uniform samples
   5 - kill amazon jobs (if 4 or 7 misconfigured)
   6 - collect results from 4
   7 - run adaptive sampling
   8 - collect results from 7
   9 - sync results to s3
   10 - sync geo locally
   11 - find missing delta geo
   12 - compute delta geo remotely (must run after 11)
   13 - collect results of delta computation
   14 - optimize cache
   15 - collect stats
   30 - convert geometry to ply format
   99 - check status

-n number workers to use

-l if set, runs locally"


THIS_CMDLINE=$@

while getopts ':hs:ln:' option; do
    case "$option" in
        h) echo "$USAGE"
            exit
            ;;
        s) STAGES=$OPTARG
            echo "Setting stages to $OPTARG"
            ;;
        l) LOCAL=1
            echo "Running locally"
            ;;
        n) N_WORKERS=$OPTARG
            echo "Setting numworkers to $OPTARG"
            ;;
        \?) printf "ERROR! illegal option: -%s\n" "$OPTARG" >&2
            echo "$USAGE" >&2
            exit 1
            ;;
    esac
done
shift $((OPTIND - 1))


# START OF ACTIONS -------------------------------------------------------------

if [ $LOCAL -ne 0 ]; then
    AWS_HOME=$LOCAL_HOME
    N_WORKERS=1
fi
AWS_PRECOMPUTE_HOME=$AWS_HOME/Documents/3DModels/templates/precomp
AWS_RUN_DIR=$AWS_PRECOMPUTE_HOME/$TPL_NAME/$RUN_ID
AWS_TMP_DIR=$AWS_RUN_DIR/tmp


# To prevent easy errors below (easy to confuse home_dir and run_dir)
unset S3_HOME_DIR
PRECOMPUTE_HOME=$LOCAL_HOME_DIR
unset LOCAL_HOME_DIR

echo "Creating directories"
# TODO: add a clean step?
mkdir -p $LOCAL_RUN_DIR
mkdir -p $LOCAL_TMP_DIR
mkdir -p $LOCAL_SCRIPT_DIR
rm -rf $LOCAL_TMP_DIR/*

touch /tmp/dummy
aws s3 cp /tmp/dummy $S3_TMP_DIR/
#aws s3 rm $S3_TMP_DIR/*

echo "--> Ok: "
RUN_INFO="    TPL = $TPL \n
    RUN_ID = $RUN_ID \n
    UNI_N_SUBDIVISIONS = $UNI_N_SUBDIVISIONS \n
    ADAPT_DEPTH = $ADAPT_DEPTH \n
    N_WORKERS = $N_WORKERS \n
    S3_RUN_DIR = $S3_RUN_DIR \n
    S3_TMP_DIR = $S3_TMP_DIR \n
    LOCAL_RUN_DIR = $LOCAL_RUN_DIR \n
    LOCAL_TMP_DIR = $LOCAL_TMP_DIR \n
    AWS_TMP_DIR = $AWS_TMP_DIR \n"
echo -e $RUN_INFO

cat >> $LOCAL_HOME/Documents/Coding/rootrepo/fab/geometry/template/precomp/script_hist.log <<EOF
$THIS_CMDLINE
$RUN_INFO

EOF

cat >> $LOCAL_RUN_DIR/run_info.log <<EOF
$THIS_CMDLINE
$RUN_INFO

EOF

# STEP 1 -----------------------------------------------------------------------

if [[ "$STAGES" == *:0:* ]]; then
    echo "Building binaries"
    cmake .
    make hadoop_mapper_main precomp_main aggregate_hybrid_main \
        prune_cache_main mesh_convert missing_delta_geo_main rm_dupes_main \
        optimize_cache_main timing_stats_main
    echo "--> Ok"
fi

# Run this if need rebuilding
AWS_CODING=$AWS_HOME/Documents/Coding
AWS_BUILD_DIR=${AWS_CODING}/build/rootrepo/fab/geometry/template/precomp
if [[ "$STAGES" == *:1:* ]]; then
    echo "Updating S3 Binaries"
    if [ $LOCAL -eq 0 ]; then
        CUR_DIR=`pwd`
        cd $EC2_HOME/..

        echo -n "Should launch an instance [y/n]? "
        read ANSWER
        if [ "$ANSWER" = "y" ]; then
            echo "Launching EC2 instance"
            ec2-run-instances $CURRENT_AMI_ID -t m3.xlarge -k my-key-pair -g shumash-security-group
        fi
        ec2-describe-instances | grep INSTANCE | grep $CURRENT_AMI_ID | \
            grep "running" | awk '{print $4;}'

        echo -n "Enter instance hostname: "
        read AWS_HOST

        aws s3 rm $S3_BINARIES/*

        echo "Now run the following: "
        echo "cd Documents/Coding/rootrepo/ && git pull origin master"
        echo "cmake . && make clean && make hadoop_mapper_main && make compute_missing_delta_geo_main"
        echo "gzip -c $AWS_BUILD_DIR/hadoop_mapper_main > /tmp/hadoop_mapper_main.gz && aws s3 cp /tmp/hadoop_mapper_main.gz $S3_BINARIES/ && gzip -c $AWS_BUILD_DIR/compute_missing_delta_geo_main > /tmp/compute_missing_delta_geo_main.gz && aws s3 cp /tmp/compute_missing_delta_geo_main.gz $S3_BINARIES/ && logout"
        ssh -i `pwd`/my-key-pair.pem admin@${AWS_HOST}

        echo "--> Ok (presumably): $S3_BINARIES"
        aws s3 ls $S3_BINARIES/
    else
        echo "--> SKIPPING (b/c running LOCALLY)"
    fi
fi


# STEP 2 -----------------------------------------------------------------------
UNI_SAMPLES_FILE="uniform_samples.txt"
CONTROL_NAMES_FILE="control_names.txt"
if [[ "$STAGES" == *:2:* ]]; then
    echo "Running uniform sampling"
    $BUILD_DIR/rootrepo/fab/geometry/template/precomp/precomp_main \
        --tpl_filename=$TPL \
        --output_file=$LOCAL_RUN_DIR/$UNI_SAMPLES_FILE \
        --control_names_out_file=$LOCAL_RUN_DIR/$CONTROL_NAMES_FILE \
        --n_divisions=$UNI_N_SUBDIVISIONS \
        --logtostderr \
        --v=2
    echo "--> Ok: $LOCAL_RUN_DIR/$UNI_SAMPLES_FILE"
fi

# STEP 3 -----------------------------------------------------------------------
if [[ "$STAGES" == *:3:* ]]; then
    echo "Copying sampling and the tpl to S3"
    cp $TPL $LOCAL_RUN_DIR/.
    aws s3 sync $LOCAL_RUN_DIR $S3_RUN_DIR
    echo "--> Ok: $S3_RUN_DIR"
fi


# STEP 4 -----------------------------------------------------------------------
CONTROL_NAMES=`cat $LOCAL_RUN_DIR/$CONTROL_NAMES_FILE`
GEO_OUTPUT_DIR=$S3_RUN_DIR/geo
UNI_CHUNK_RES_NAME=uniform_result.txt
UNI_GEO_RES_NAME=uniform_geo_cache.ascii_proto
N_CHUNKS=$((N_WORKERS-1))

AWS_MAPPER_BIN=${AWS_BUILD_DIR}/hadoop_mapper_main
if [[ "$STAGES" == *:4:* ]]; then
    # Note: we could not run Hadoop, because it was not available for custom
    # AMIs on Amazon, and we require a dozen libraries installed for the code to run
    echo "Running Makeshift MR on uniform samples"

    if [ $LOCAL -eq 0 ]; then
        cd $EC2_HOME/..
        get_hosts
    fi

    echo "Creating worker script"
    cat > $LOCAL_TMP_DIR/worker_script.sh <<EOF
#!/bin/bash
set -o nounset

mkdir -p $AWS_TMP_DIR
mkdir -p $AWS_RUN_DIR/geo
LOG_FILE=$AWS_TMP_DIR/uni_worker.log-\${1}
RES_FILE=$AWS_RUN_DIR/${UNI_CHUNK_RES_NAME}-\${1}
GEO_RES_FILE=$AWS_RUN_DIR/${UNI_GEO_RES_NAME}-\${1}

echo "" > \$RES_FILE
echo "" > \$GEO_RES_FILE
echo "" > \$LOG_FILE

touch /tmp/uni_START_\${1}_\${2}
aws s3 cp /tmp/uni_START_\${1}_\${2} $S3_TMP_DIR/

EOF
    if [ $LOCAL -eq 0 ]; then
        cat >> $LOCAL_TMP_DIR/worker_script.sh <<EOF
rm ${AWS_MAPPER_BIN}
aws s3 cp $S3_BINARIES/hadoop_mapper_main.gz /tmp/hadoop_mapper_main.gz > \$LOG_FILE 2>&1
gunzip -c /tmp/hadoop_mapper_main.gz  > ${AWS_MAPPER_BIN} 2>> \$LOG_FILE
chmod a+x ${AWS_MAPPER_BIN}

EOF
    fi

    cat >> $LOCAL_TMP_DIR/worker_script.sh <<EOF
RET_CODE=1
FAILURES=0
until [ "\${RET_CODE}" -eq "0" ]; do
echo "Running mapper" >> \$LOG_FILE
cat $AWS_RUN_DIR/$UNI_SAMPLES_FILE-\${1} | \\
${AWS_MAPPER_BIN} \\
--strict_geometry_exceptions=true \\
--enable_triangle_logging=false \\
--enable_analytics=true \\
--volumetric_csg_fallback=false \\
--logtostderr=true \\
--fresh_recompute_every_time=true \\
--v=1 \\
--action=UNI \\
--restart=true \\
--cache_all_roots=false \\
--data_path="$AWS_CODING/rootrepo/data" \\
--caching_ms_threshold=100 \\
--control_names=$CONTROL_NAMES \\
--tpl_filename=$S3_RUN_DIR/$(basename $TPL) \\
--geometry_out_dir=$GEO_OUTPUT_DIR \\
--local_geometry_out_dir=$AWS_RUN_DIR/geo \\
--geometry_index_out_file=\${GEO_RES_FILE} \\
--samples_out_file=\$RES_FILE \\
--hack_sim_disable_memory_freeing=false \\
--aws_access_key=$AWS_ACCESS_KEY \\
--hack_copy_sim_mesh=false \\
--aws_secret_key=$AWS_SECRET_KEY >> \$LOG_FILE 2>&1
RET_CODE="\$?"

if [ "\$RET_CODE" -ne "0" ]; then
  let FAILURES+=1
  FAIL_INFO_FILE="/tmp/uni_FAIL_\${1}_\${2}"
  echo "FAILURE # \$FAILURES" > \$FAIL_INFO_FILE
  echo "FAILED EXAMPLE: " >> \$FAIL_INFO_FILE
  tail -n1 \$RES_FILE >> \$FAIL_INFO_FILE
  echo "--------- HEAD -----------: " >> \$FAIL_INFO_FILE
  head -n200 \$LOG_FILE >> \$FAIL_INFO_FILE
  echo "--------- TAIL -----------: " >> \$FAIL_INFO_FILE
  tail -n200 \$LOG_FILE >> \$FAIL_INFO_FILE
  aws s3 cp \$FAIL_INFO_FILE $S3_TMP_DIR/
  aws s3 cp \$LOG_FILE $S3_TMP_DIR/
fi
done

aws s3 cp \$LOG_FILE $S3_TMP_DIR/
aws s3 cp \$RES_FILE $S3_RUN_DIR/
aws s3 cp \$GEO_RES_FILE $S3_RUN_DIR/

echo "" >> /tmp/uni_OK_\${1}_\${2}
aws s3 cp /tmp/uni_OK_\${1}_\${2} $S3_TMP_DIR/

aws s3 sync $GEO_OUTPUT_DIR $S3_RUN_DIR/geo
if [ "\$?" -eq "0" ]; then
  echo "" >> /tmp/uni_OK_GEO_\${1}_\${2}
  aws s3 cp /tmp/uni_OK_GEO_\${1}_\${2} $S3_TMP_DIR/
fi

aws ec2 stop-instances --instance-ids \$(aws ec2 describe-instances --query 'Reservations[*].Instances[*].[PublicDnsName, State.Name, InstanceId]' --output text | grep "\${2}" | awk '{printf \$NF; }')

EOF
    chmod a+x $LOCAL_TMP_DIR/worker_script.sh
    echo "--> Ok: $LOCAL_TMP_DIR/worker_script.sh"

    if [ $N_CHUNKS -eq 0 ]; then
        CH_SIZE=$(wc -l $LOCAL_RUN_DIR/$UNI_SAMPLES_FILE | awk '{printf "%d", $1;}')
    else
        CH_SIZE=$(wc -l $LOCAL_RUN_DIR/$UNI_SAMPLES_FILE | awk "{s=\$1 / $N_CHUNKS; printf \"\%d\", int(s);}")
    fi
    echo "Computed N_CHUNKS: $N_CHUNKS CH_SIZE: $CH_SIZE"

    for CH_N in $(seq 0 $N_CHUNKS); do #0 4); do #$N_CHUNKS); do
        echo ""

        CHUNK=$LOCAL_RUN_DIR/$UNI_SAMPLES_FILE-${CH_N}
        echo "Creating CHUNK $CH_N"
        cat $LOCAL_RUN_DIR/$UNI_SAMPLES_FILE | \
            awk "BEGIN{n=0}{n=n+1; if (n >= $CH_N * $CH_SIZE && ($CH_N == $N_CHUNKS || n < ($CH_N + 1) * $CH_SIZE)) {print;}}" \
            > $CHUNK
        echo "--> Ok: $CHUNK"

        if [ $LOCAL -eq 0 ]; then
            AWS_HOST=$(host_number $CH_N)
            echo "Setting up host $AWS_HOST"
            ssh -oStrictHostKeyChecking=no -t -t -i `pwd`/my-key-pair.pem \
                admin@${AWS_HOST} "mkdir -p $AWS_RUN_DIR && exit"
            scp -oStrictHostKeyChecking=no -i `pwd`/my-key-pair.pem \
                $CHUNK admin@${AWS_HOST}:$AWS_RUN_DIR/
            scp -oStrictHostKeyChecking=no -i `pwd`/my-key-pair.pem \
                $LOCAL_TMP_DIR/worker_script.sh admin@${AWS_HOST}:$AWS_RUN_DIR/

            ssh -oStrictHostKeyChecking=no -t -t -i `pwd`/my-key-pair.pem admin@${AWS_HOST} <<EOF
${AWS_RUN_DIR}/worker_script.sh ${CH_N} ${AWS_HOST} &
ps -ef | grep "worker_script"
exit
EOF
        else
            echo "Running LOCALLY"
            $LOCAL_TMP_DIR/worker_script.sh $CH_N "LOCAL"
        fi
    done
fi

################################################################################

# About 700 more lines have been removed.
#
# Most certainly a giant shell script is not a good way to go in general, but it
# was the most time-efficient way to integrate with AWS for the purpose of this
# project.

################################################################################

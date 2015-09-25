#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_STATS_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_STATS_H__

#include <string>
#include <utility>
#include <vector>
#include <set>
using std::pair;
using std::vector;
using std::string;
using std::set;

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <Eigen/Dense>

namespace mit_plato {

class PrecompLookup;
class ControlValues;
class LookupStatsProto;

namespace util {

// Wrapper over Eigen::MatrixXd, exposing [][] interface required
// by boost graph algorithms.
class MatrixWrap {
 public:
  class RowWrap {
    const MatrixWrap& parent;
    int x;

   public:
    RowWrap(const MatrixWrap& p, int in_x) : parent(p), x(in_x) {}

    const double& operator[](int y) const { return parent.data_(x, y); }
    double& operator[](int y) { return parent.data_(x, y); }
  };

  MatrixWrap(Eigen::MatrixXd& data) : data_(data) {}

  RowWrap operator[](int x) { return RowWrap(*this, x);}

  const RowWrap operator[](int x) const { return RowWrap(*this, x);}

 private:
  friend class RowWrap;

  Eigen::MatrixXd& data_;
};

}

// Encapsulates various statistics over the precomputed design space
class LookupStats {
 public:
  struct VertexProperties {
    VertexProperties()
        : sample(NULL),
          dist(std::numeric_limits<double>::max()),
          cluster(-1) {}
    const ControlValues* sample;

    double dist;
    int cluster;
  };

  struct GraphStats {
    GraphStats() : radius(0), size(0), seed_sample(NULL), center_sample(NULL) {}
    GraphStats(double _radius, int _size, const ControlValues* _seed, const ControlValues* _center)
        : radius(_radius), size(_size), seed_sample(_seed),center_sample(_center) {}
    double radius;
    int size;
    const ControlValues* center_sample;
    const ControlValues* seed_sample;
  };

  typedef boost::adjacency_list<
    boost::setS,
    boost::vecS,
    boost::undirectedS,
    VertexProperties,
    boost::property<boost::edge_weight_t, double> >
  GraphType;

  typedef boost::graph_traits<GraphType>::vertex_descriptor
  VertexDesc;
  typedef boost::graph_traits<GraphType>::edge_descriptor
  EdgeDesc;
  typedef boost::property_map<GraphType, int VertexProperties::*>::const_type
  ConstVertexPropMap;

  LookupStats(const PrecompLookup& lookup);

  LookupStats(const PrecompLookup& lookup,
              const LookupStatsProto& spec);

  ~LookupStats();

  void ToSpec(LookupStatsProto* spec) const;

  int ComputeConnectedComponents();
  void ComputeComponentCentroids();

  const ControlValues* ComputeApproximateCentroid(const GraphType* graph,
                                                  double* radius) const;

  const ControlValues* ComputeExactCentroid(const GraphType* graph,
                                            double* radius) const;

  const GraphType& graph() const { return graph_; }
  const vector<GraphType*>& components() const { return comps_; }
  const vector<GraphStats>& component_stats() const { return comp_stats_; }

  // Helper for constructing graphs between sample points
  class GraphBuilder {
   public:
    GraphBuilder(GraphType& graph) : graph_(graph) {}

    void Build(const PrecompLookup& lookup,
               vector<pair<const ControlValues*, const ControlValues*> >* missing_deltas = NULL);

    VertexDesc add_or_get_vertex(const ControlValues* s);

    EdgeDesc add_edge(VertexDesc v1, VertexDesc v2, double weight);

   private:
    GraphType& graph_;
    std::map<const ControlValues*, VertexDesc> verts_;
  };
 private:

  static void SplitIntoComponentGraphs(const GraphType& graph,
                                       int n_components,
                                       const vector<int>& comp_verts,
                                       vector<GraphType*>* comps);

  void PopulateSubcomponentGraphs();

  const PrecompLookup& lookup_;
  GraphType graph_;
  vector<GraphType*> comps_;
  vector<GraphStats> comp_stats_;
};

std::ostream& operator<<(std::ostream& os,
                         const LookupStats::GraphStats& stats);

// Partitions the graph of valid samples using furthest point sampling
// using Change in Geometry as the distance on the graph edges
class FurthestPointGraphPartition {
 public:
  typedef LookupStats::GraphType GraphType;
  typedef LookupStats::VertexDesc VertexDesc;

  FurthestPointGraphPartition(GraphType& g);

  void Partition(int n_samples,
                 vector<VertexDesc>* samples,
                 int min_cluster_size,
                 int max_cluster_size);

  void Partition(int n_samples,
                 vector<VertexDesc>* samples);
 private:
  // Function for updating the Voronoi diagram over the graph
  // given the addition of a new sample
  void InitializeTentativeDistances(VertexDesc new_sample);
  bool is_unvisited(VertexDesc v) const;
  void mark_as_ignored(VertexDesc v);
  void mark_as_visited(VertexDesc v);

  int largest_partition() const;
  int num_good_samples(int min_partition_size) const;

  // Implementation of Djikstra's single source distances algorithm,
  // adapted to stop updates once the boundary of the voronoi cell
  // is reached.
  // Returns true if current vertex is part of the new pratition;
  // if not bake_in_distances has no effect on the current voronoi diagram
  bool ProcessCurrentVertex(VertexDesc current, bool bake_in_distances = true);
  VertexDesc GetCurrentVertex() const;
  int EstimateNewPartitionSize(VertexDesc new_sample);

  // Functions for adding seed sample to partition
  void AddSample(VertexDesc new_sample);
  VertexDesc PickNewSample() const;
  VertexDesc RandomVertex() const;
  VertexDesc FurthestVertex() const;
  VertexDesc RandomFarVertex() const;
  VertexDesc TryToFindSampleWithGoodPartition(int min_cluster_size);

  GraphType& graph_;
  vector<VertexDesc> samples_;
  vector<int> partition_sizes_;

  // Temporary variables used for the distance updates
  set<VertexDesc> unvisited_;
  vector<double> tmp_dist_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_STATS_H__

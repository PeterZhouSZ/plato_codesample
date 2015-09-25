#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_GEOMETRY_CACHE_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_GEOMETRY_CACHE_H__

#include <ostream>
#include <string>
#include <map>
#include <list>
#include <set>
#include <vector>
#include <utility>
using std::string;
using std::map;
using std::vector;
using std::set;
using std::list;

//#define PLATO_ENABLE_THREADS
#ifdef PLATO_ENABLE_THREADS
#include <boost/thread.hpp>
#endif

#include "fab/geometry/tri_mesh.h"
#include "fab/geometry/template/template.pb.h"
#include "fab/geometry/template/template.h"

namespace mit_plato {

class Shape;
class FabGeometryTemplate;
class TemplateParams;

// GEOMETRY LOOKUP -------------------------------------------------------------

class GeometryLookup {
 public:
  GeometryLookup();
  ~GeometryLookup();

  GeometryLookup(const SubtreesCacheSpec& cache,
                 const string& file_prefix,
                 const string& remote_file_prefix = "");

  void ToSummarySpec(SubtreesCacheSpec* spec) const;

  void AddData(const SubtreesCacheSpec& cache);

  void AddCachedSubtree(const SubtreeCachingInfo& subtree);

  bool HasCachedSubtree(const SubtreeCachingInfo& subtree) const;

  //! Inserts (if exists) chached subtrees for all the subtrees that
  //! are dirty. If the parent gets a cache hit, childern are never
  //! evalutated.
  //  void InsertCachedSubtrees(OperationNode* node) const;
  #ifdef PLATO_ENABLE_THREADS
  void InsertCachedSubtrees(OperationNode* node,
                            boost::thread_group* threads) const;
  #else
  void InsertCachedSubtrees(OperationNode* node) const;
  #endif

  //! As above, but inserts cached subtrees for all roots of the template.
  void InsertCachedSubtrees(FabGeometryTemplate* tpl) const;

  Shape* GetCachedGeometry(const string& node_key) const;

  Shape* GetCachedGeometryFromFile(const string& file) const;

  const SubtreeCachingInfo* GetInfo(const string& node_key) const;

  int size() const { return cache_.size(); }

  bool TestResolveFile() const;

  void LogInfo() const;

  //! Sets up Mock Pruned cache
  void InitOptimizedCache(const string& available_keys_file = "");

  // HACK
  static void CorrectKey(string* key);

  bool IsKeyAvailable(const string& key) const;

 private:
  void PrefetchLargestMeshes(const double bytes_on_disk);

  bool GetCachedFile(const string& node_key,
                     string* filename) const;

  void LoadCachedGeometryTask(OperationNode* node,
                              const string& filename) const;

  SubtreeCachingInfo* GetInfo(const string& node_key);

  void SetAvailableKeys(const string& available_keys_file = "");

  // Meshes slowest to load are pre-fetched
  // file : TriMesh*
  map<string, const TriMesh*> prefetched_;

  // Full key : file cache
  map<string, string> cache_;

  // Full key : info cache
  map<string, SubtreeCachingInfo*> cache_info_;

  // Keys sorted by score = t * x (all keys)
  //vector<std::pair<double, string> > sorted_keys_;
  vector<std::pair<double, string> > sorted_sizes_;

  // Available keys
  set<string> available_keys_;

  // Absolute path prefix for the filenames in cache
  string file_prefix_;

  // Remote file location (if downloading on demand)
  string remote_file_prefix_;
};

// Dependency Graph ------------------------------------------------------------

class GeometryDependencyGraph {
 public:
  GeometryDependencyGraph() : valid_samples_(0) {}

  GeometryDependencyGraph(const GeoDependencyGraphSpec& spec);

  ~GeometryDependencyGraph();

  // Adds metadata for select nodes
  void AddMetadata(const SubtreesCacheSpec& spec);

  // Adds current subtrees to graph
  void AddToGraph(const FabGeometryTemplate& instance,
                  const string& sample_tag = "");

  int valid_samples() const { return valid_samples_; }

  // DepNode -----------------------------------------------
  // Encodes a subtree node in the geometry dependency
  struct DepNode {
    DepNode(const string& key) : n_evals(0), ave_ms(0), memory(0), key(key) {}

    bool has_stats() const { return n_evals > 0; }

    int GetNevals() const {
      if (has_stats()) { return n_evals; }

      int sum = 0;
      for (const DepNode* parent : parents) {
        CHECK(parent);
        sum += parent->GetNevals();
      }
      return sum;
    }

    bool IsCachable() const {
      return memory > 0.1;
    }

    int getMemoryKB() const {
      return floor(memory / 1000);
    }

    void ToSpec(const map<const DepNode*, int>& ids,
                GeoDependencyGraphSpec::Node* spec) const;

    void FillFromSpec(const map<int, DepNode*>& nodes,
                      const GeoDependencyGraphSpec::Node& spec);

    bool IsSane() const {
      for (const DepNode* parent : parents) {
        if (parent->children.find(const_cast<DepNode*>(this)) == parent->children.end()) {
          LOG(ERROR) << "Parent misses child reference!!";
          LOG(ERROR) << "Child: " << key;
          LOG(ERROR) << "Parent: " << parent->key;
          return false;
        }
      }
      for (const DepNode* child : children) {
        if (child->parents.find(const_cast<DepNode*>(this)) == child->parents.end()) {
          LOG(ERROR) << "Child misses parent reference!!";
          LOG(ERROR) << "Child: " << child->key;
          LOG(ERROR) << "Parent: " << key;
        }
      }
      if (parents.empty()) {
        return true;
      }

      int parent_evals = 0;
      for (const DepNode* parent : parents) {
        parent_evals += parent->GetNevals();
      }
      if (parent_evals != GetNevals()) {
        LOG(ERROR) << "Parent evals != GetNevals() "
                   << parent_evals << " != " << GetNevals();
        LOG(ERROR) << "Key: " << key;
        return false;
      }
      return true;
    }

    set<DepNode*> parents;
    set<DepNode*> children;
    int n_evals;
    double ave_ms;
    double memory;
    string key;
    set<string> samples;
  };
  // -------------------------------------------------------

  // Returns node for a given key (or NULL if none)
  const DepNode* GetNode(const string& key) const;

  DepNode* GetNode(const string& key);

  const set<DepNode*>& roots() const { return roots_; }

  const map<string, DepNode*>& nodes() const { return nodes_; }

  bool is_root(const DepNode* n) const {
    return roots_.find(const_cast<DepNode*>(n)) != roots_.end();
  }

  void LogStats() const;

  string StatsString() const {
    std::stringstream ss;
    ss <<  nodes_.size() << " nodes, " << roots_.size() << " roots";
    return ss.str();
  }

  void ToSpec(GeoDependencyGraphSpec* spec) const;

 private:

  // Recursively adds node and its children to the graph,
  // only adding new edges if some of the nodes already exist
  DepNode* AddToGraph(const OperationNode* opNode,
                      DepNode* parent = NULL,
                      const string& sample_tag = "");

  // Adds a node with a particular key and sets parent pointers;
  // simly adds new edges if the node already exists
  DepNode* AddToGraph(const string& node_key, DepNode* parent);

  // Finds or creates and adds a new node
  DepNode* GetOrMakeNode(const string& key);

  // All the nodes in the graph
  map<string, DepNode*> nodes_;
  set<DepNode*> roots_;
  int valid_samples_;
};

// Cache optimizer -------------------------------------------------------------

class CacheOptimizer {
 public:
  typedef GeometryDependencyGraph::DepNode DepNode;

  CacheOptimizer(const GeometryDependencyGraph& dep_graph);

  // Stats -------------------------------------------------
  double memory() const {
    return memory_;
  }

  double ave_compute_time() const;

  double total_compute_time() const;

  double worst_compute_time() const;

  double stdev_compute_time() const;

  int num_touched_samples() const;

  int num_cached_nodes() const {
    return cached_.size();
  }

  bool AllCached() const {
    return sorted_.empty();
  }

  // Optimization methods ----------------------------------
  void GreedyIterationAveTime();

  void GreedyIterationMaxTime();

  void NaiveCacheNext();

  const set<GeometryDependencyGraph::DepNode>& cache() const;

  // Auxiliary ---------------------------------------------
  bool IsInCache(const DepNode* n) const;

  bool IsPathCached(const DepNode* n) const;

  double FractionUncached(const DepNode* n) const;
  double FractionParentUncached(const DepNode* n) const;

  double ScoreFunction(const int n_evals,
                       const double eval_ms,
                       const double frac_uncached = 1.0) const;

  // Output -------------------------------------------------
  void WriteUsedKeys(std::ostream& os) const;

 private:
  void GetUncachedSubtrees(const DepNode* parent,
                           set<const DepNode*>* candidates) const;

  void RecacheSubtree(const DepNode* n, double current_score);

  double GetAdjustedScore(const DepNode* n) const;

  void AddToCache(const DepNode* n);

  void RemoveFromCache(const DepNode* n);

  void ReinsertCandidate(double adjusted_score, const DepNode* n);

  double EstimateComputeTime(const DepNode* n) const;

  void RecomputeStats(const DepNode* changed_cache_item);

  void AddTouchesToSample(const string& sample, int touches);

  void InitComputeTime() const;

  void GetRoots(const DepNode* n,
                set<const DepNode*>* roots) const;

  list<std::pair<double, const DepNode*> > sorted_;
  set<const DepNode*> cached_;
  const GeometryDependencyGraph& graph_;
  double memory_;

  mutable map<const DepNode*, double> fraction_uncached_;
  mutable map<const DepNode*, double> root_compute_;
  mutable double total_compute_time_;
  mutable double compute_time_squares_;
  mutable double worst_time_;
  map<string, int> touched_samples_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_GEOMETRY_CACHE_H__

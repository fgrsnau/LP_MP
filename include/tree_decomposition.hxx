#ifndef LP_MP_TREE_DECOMPOSITION_HXX
#define LP_MP_TREE_DECOMPOSITION_HXX

#include "LP_MP.h"
#include "serialization.hxx"
#include "union_find.hxx"

namespace LP_MP {

// given factors connected as a tree, solve it.
template<typename FMC>
class factor_tree {
public:
   // add messages from leaves to root upward
   template<typename MESSAGE_CONTAINER_TYPE>
   void add_message( MESSAGE_CONTAINER_TYPE& msg, Chirality c) // chirality denotes which factor is nearer the root
   {

   }

   void init()
   {
       assert(false);
      /*
      std::set<FactorTypeAdapter*> factor_set;
      for(auto tree_msg : tree_messages_) {
         auto* msg = std::get<0>(tree_msg);
         factor_set.insert(msg->GetLeftFactor());
         factor_set.insert(msg->GetRightFactor());
      }
      factors_.assign( factor_set.begin(), factor_set.end() );
      std::sort(factors_.begin(), factors_.end());
      assert(factors_.size() == tree_messages_.size() + 1);

      assert(tree_valid());
      */
   }

   // check whether messages are arranged correctly
   bool tree_valid() const
   {
       assert(false);
       /*
     // build graph out of tree messages and check whether all edges point towards root
     std::vector<std::vector<int>> tree(factors_.size());
     std::unordered_map<FactorTypeAdapter*, int> factor_map;
     for(INDEX i=0; i<factors_.size(); ++i) {
       factor_map.insert({ factors_[i], i });
     }
     for(auto& tree_msg : tree_messages_) {
         auto* msg = std::get<0>(tree_msg);
         auto c = std::get<1>(tree_msg);
         auto* left = msg->GetLeftFactor();
         const int left_idx = factor_map[left];
         auto* right = msg->GetRightFactor();
         const int right_idx = factor_map[right];
         // point messages here towards leaves
         if(c == Chirality::left) {
           tree[left_idx].push_back(right_idx); 
         } else {
           assert(c == Chirality::right);
           tree[right_idx].push_back(left_idx); 
         } 
     }
     // determine root: it is the single index without incoming edges
     std::vector<int> no_incoming_edges(factors_.size(), 0);
     for(INDEX i=0; i<tree.size(); ++i) {
       for(int j : tree[i]) {
         no_incoming_edges[j]++;
       }
     }
     assert(1 == std::count(no_incoming_edges.begin(), no_incoming_edges.end(), 0));
     // check if tree is connected
     UnionFind uf(tree.size());
     for(INDEX i=0; i<tree.size(); ++i) {
       for(int j : tree[i]) {
         uf.merge(i,j);
       }
     }
     assert(uf.count() == 1);

     const int root = std::find(no_incoming_edges.begin(), no_incoming_edges.end(), 0) - no_incoming_edges.begin();

     // check whether messages are in correct order: from bottom to top
     std::vector<int> visited(tree.size(), false);
     for(auto& tree_msg : tree_messages_) {
         auto* msg = std::get<0>(tree_msg);
         auto c = std::get<1>(tree_msg);
         auto* left = msg->GetLeftFactor();
         const int left_idx = factor_map[left];
         auto* right = msg->GetRightFactor();
         const int right_idx = factor_map[right];
         // point messages here towards leaves
         if(c == Chirality::left) {
           visited[right_idx] = true;
           assert(visited[left_idx] == false);
         } else {
           visited[left_idx] = true;
           assert(visited[right_idx] == false); 
         }
     }
     assert(visited[root] == false);

     return true; 
       */
   }

   REAL solve()
   {
       assert(false);
       /*
      assert(factors_.size() == tree_messages_.size() + 1); // otherwise call init
      // send messages up the tree
      for(auto it = tree_messages_.begin(); it!= tree_messages_.end(); ++it) {
         auto* m = std::get<0>(*it);
         Chirality c = std::get<1>(*it);
         m->send_message_up(c);
      }
      // compute primal for topmost factor
      // also init primal for top factor, all other primals were initialized already by send_message_up
      REAL value = 0.0;
      {
         auto* msg = std::get<0>(tree_messages_.back());
         Chirality c = std::get<1>(tree_messages_.back());
         if(c == Chirality::right) {
            // init primal for right factor!
            msg->GetRightFactorTypeAdapter()->init_primal();
            msg->GetRightFactorTypeAdapter()->MaximizePotentialAndComputePrimal();
            value = msg->GetRightFactorTypeAdapter()->EvaluatePrimal();
         } else {
            assert(c == Chirality::left);
            msg->GetLeftFactorTypeAdapter()->init_primal(); 
            msg->GetLeftFactorTypeAdapter()->MaximizePotentialAndComputePrimal(); 
            value = msg->GetLeftFactorTypeAdapter()->EvaluatePrimal();
         }
      }
      // track down optimal primal solution
      for(auto it = tree_messages_.rbegin(); it!= tree_messages_.rend(); ++it) {
         auto* msg = std::get<0>(*it);
         Chirality c = std::get<1>(*it);
         msg->track_solution_down(c);
         if(c == Chirality::right) {
            value += msg->GetLeftFactorTypeAdapter()->EvaluatePrimal();
         } else {
            assert(c == Chirality::left);
            value += msg->GetRightFactorTypeAdapter()->EvaluatePrimal();
         }
      } 

      // check if primal cost is equal to lower bound
      assert(primal_consistent());
      assert(std::abs(lower_bound() - primal_cost()) <= eps);
      assert(std::abs(value - primal_cost()) <= eps);
      return value;
       */
   }

   bool primal_consistent() const 
   {
       /*
      for(auto it = tree_messages_.begin(); it!= tree_messages_.end(); ++it) {
         auto* m = std::get<0>(*it);
         if(!m->CheckPrimalConsistency()) {
            return false;
         }
      }
      return true; 
      */
   }

   REAL primal_cost() const
   {
      REAL cost = 0.0;
      for(auto* f : factors_) {
         cost += f->EvaluatePrimal();
         if(cost == std::numeric_limits<REAL>::infinity()) {
            assert(false);
         }
      }
      return cost;
   }

   REAL lower_bound() const 
   {
      REAL lb = 0.0;
      for(auto* f : factors_) {
         lb += f->LowerBound();
      }
      return lb;
   }

   template<typename FACTOR_TYPE>
   std::vector<FACTOR_TYPE*> get_factors() const
   {
      std::vector<FACTOR_TYPE*> factors;
      std::set<FACTOR_TYPE*> factor_present;
      for(auto& t : tree_messages_) {

         auto* left = std::get<0>(t)->GetLeftFactorTypeAdapter();
         auto* left_cast = dynamic_cast<FACTOR_TYPE*>(left);
         if(left_cast && factor_present.find(left_cast) == factor_present.end()) {
            factors.push_back(left_cast);
            factor_present.insert(left_cast);
         }

         auto* right = std::get<0>(t)->GetRightFactorTypeAdapter();
         auto* right_cast = dynamic_cast<FACTOR_TYPE*>(right);
         if(right_cast && factor_present.find(right_cast) == factor_present.end()) {
            factors.push_back(right_cast);
            factor_present.insert(right_cast);
         } 
      }
      return factors;
   }

   struct free_message_container {
      template<class MESSAGE_CONTAINER_TYPE>
         using invoke = typename MESSAGE_CONTAINER_TYPE::free_message_container_type;
   };
   using free_message_container_type_list = meta::transform< typename FMC::MessageList, free_message_container >;
   using free_message_variant = meta::apply<meta::quote<std::variant>, free_message_container_type_list>;
   std::vector< std::tuple<free_message_variant, Chirality> > tree_messages_;

   std::vector<FactorTypeAdapter*> factors_;

protected:
};

// factors can be shared among multiple trees. Equality between shared factors is enforced via Lagrangean multipliers
class Lagrangean_factor_base
{
public:
  Lagrangean_factor_base(FactorTypeAdapter* factor)
    : f(factor),
    no_Lagrangean_vars_(factor->dual_size())
  {}

  INDEX no_Lagrangean_vars() const
  {
    assert(no_Lagrangean_vars_ == f->dual_size());
    return no_Lagrangean_vars_; 
  }

  void serialize_Lagrangean(const double* w, const double scaling)
  {
    serialization_archive ar(w, no_Lagrangean_vars_*sizeof(REAL));
    addition_archive l_ar(ar, 1.0*scaling);
    f->serialize_dual(l_ar);
    ar.release_memory(); 
  }
  //void copy_fn(double* w)
  //{
  //  f->subgradient(w, +1.0); 
  //}
  //REAL dot_product_fn(double* w)
  //{
  //  return f->dot_product(w);
  //}

  FactorTypeAdapter* f;
  const INDEX no_Lagrangean_vars_;
  INDEX global_Lagrangean_vars_offset_; // at which offset are the Lagrangean variables for factor f stored?
  INDEX local_Lagrangean_vars_offset_; // in the mapped subspace, at which position do the Lagrangean variables start?
};

// the first factor collects all positive Lagrangean variables, the other one negative copy
class Lagrangean_factor_star : public Lagrangean_factor_base {
public:
  using Lagrangean_factor_base::Lagrangean_factor_base;

  static INDEX joint_no_Lagrangean_vars(const std::vector<Lagrangean_factor_star>& factors)
  {
    assert(factors.size() > 1);
    const INDEX v = factors[0].no_Lagrangean_vars();
    return v*(factors.size()-1);
  }

  static void init_Lagrangean_variables(std::vector<Lagrangean_factor_star>& factors, INDEX Lagrangean_vars_begin)
  {
    assert(factors.size() > 1);
    const INDEX v = factors[0].no_Lagrangean_vars();

    factors[0].global_Lagrangean_vars_offset_ = Lagrangean_vars_begin; 
    factors[0].no_connected = factors.size();

    for(INDEX i=1; i<factors.size(); ++i) {
      factors[i].global_Lagrangean_vars_offset_ = Lagrangean_vars_begin; 
      factors[i].no_connected = 0;
      Lagrangean_vars_begin += v;
    }
  }

  void add_to_mapping(std::vector<int>& mapping)
  {
    local_Lagrangean_vars_offset_ = mapping.size();
    if(no_connected > 0) {
      assert(no_connected > 1);
      for(INDEX i=0; i<(no_connected-1)*no_Lagrangean_vars(); ++i) {
        mapping.push_back(global_Lagrangean_vars_offset_ + i);
      }
    } else {
      local_Lagrangean_vars_offset_ = mapping.size();
      for(INDEX i=0; i<no_Lagrangean_vars(); ++i) {
        mapping.push_back(global_Lagrangean_vars_offset_ + i);
      }
    }
  }

  void serialize_Lagrangean(const double* wi, const double scaling)
  {
    const double* w = wi + local_Lagrangean_vars_offset_;
    if(no_connected > 0) {
      assert(no_connected > 1);
      for(INDEX i=0; i<no_connected-1; ++i) {
        serialization_archive ar(w + i*no_Lagrangean_vars(), Lagrangean_factor_base::no_Lagrangean_vars()*sizeof(REAL));
        addition_archive l_ar(ar, 1.0*scaling);
        f->serialize_dual(l_ar);
        ar.release_memory(); 
      }
    } else {
      serialization_archive ar(w, Lagrangean_factor_base::no_Lagrangean_vars()*sizeof(REAL));
      addition_archive l_ar(ar, -1.0*scaling);
      f->serialize_dual(l_ar);
      ar.release_memory(); 
    } 
  }

  void copy_fn(double* wi)
  {
    double* w = wi + local_Lagrangean_vars_offset_;
    if(no_connected > 0) {
      assert(no_connected > 1);
      for(INDEX i=0; i<no_connected-1; ++i) {
        f->subgradient(w + i*no_Lagrangean_vars(), +1.0); 
      }
    } else {
      f->subgradient(w, -1.0); 
    }
  }
  REAL dot_product_fn(double* wi)
  {
    double* w = wi + local_Lagrangean_vars_offset_;
    if(no_connected > 0) {
      double d = 0.0;
      for(INDEX i=0; i<no_connected-1; ++i) {
        d += f->dot_product(w + i*no_Lagrangean_vars());
      }
      return d;
    } else {
      return -f->dot_product(w);
    }
  }

protected:
  INDEX no_connected; // == 0 for negative, == no factors for positive factor. Possibly rename.
};

class Lagrangean_factor_FWMAP : public Lagrangean_factor_base {
public:
  using Lagrangean_factor_base::Lagrangean_factor_base;

  ~Lagrangean_factor_FWMAP()
  {
    static_assert(sizeof(REAL) == sizeof(double), "needed for FWMAP implementation");
  }

  static INDEX joint_no_Lagrangean_vars(const std::vector<Lagrangean_factor_FWMAP>& factors)
  {
    assert(factors.size() > 0);
    return factors[0].no_Lagrangean_vars();
  }

  static void init_Lagrangean_variables(std::vector<Lagrangean_factor_FWMAP>& factors, const INDEX Lagrangean_vars_begin)
  {
    assert(factors.size() > 0);
    for(INDEX i=0; i<factors.size(); ++i) {
      factors[i].global_Lagrangean_vars_offset_ = Lagrangean_vars_begin; 
    } 
  }

  void add_to_mapping(std::vector<int>& mapping)
  {
    local_Lagrangean_vars_offset_ = mapping.size();
    for(INDEX i=0; i<no_Lagrangean_vars(); ++i) {
      mapping.push_back(global_Lagrangean_vars_offset_ + i);
    }
  }

  void serialize_Lagrangean(const double* wi, const double scaling)
  {
    const double* w = wi + local_Lagrangean_vars_offset_;
    serialization_archive ar(w, Lagrangean_factor_base::no_Lagrangean_vars()*sizeof(REAL));
    addition_archive l_ar(ar, 1.0*scaling);
    f->serialize_dual(l_ar);
    ar.release_memory(); 
  }
  void copy_fn(double* wi)
  {
    double* w = wi + local_Lagrangean_vars_offset_;
    f->subgradient(w, +1.0); 
  }
  REAL dot_product_fn(double* wi)
  {
    double* w = wi + local_Lagrangean_vars_offset_;
    return f->dot_product(w);
  }
};

class Lagrangean_factor_zero_sum : public Lagrangean_factor_base {
public:
  using Lagrangean_factor_base::Lagrangean_factor_base;

  static INDEX joint_no_Lagrangean_vars(const std::vector<Lagrangean_factor_zero_sum>& factors)
  {
    assert(factors.size() > 0);
    const INDEX no_Lagrangean_vars = factors[0].no_Lagrangean_vars_;
    return no_Lagrangean_vars * factors.size(); 
  }
  static void init_Lagrangean_variables(std::vector<Lagrangean_factor_zero_sum>& factors, const INDEX Lagrangean_vars_begin)
  {
    assert(factors.size() > 0);
    const INDEX no_Lagrangean_vars = factors[0].no_Lagrangean_vars_;
    for(INDEX i=0; i<factors.size(); ++i) {
      factors[i].global_Lagrangean_vars_offset_ = Lagrangean_vars_begin + i*no_Lagrangean_vars; 
    } 
  }

  void add_to_mapping(std::vector<int>& mapping)
  {
    local_Lagrangean_vars_offset_ = mapping.size();
    for(INDEX i=0; i<no_Lagrangean_vars_; ++i) {
      mapping.push_back(global_Lagrangean_vars_offset_ + i);
    } 
  }
};

class Lagrangean_factors_cyclic {
public:
protected:
  FactorTypeAdapter* f;
  INDEX Lagrangean_vars_offset; // at which offset are the Lagrangean variables for factor f stored?
  INDEX dual_size_; // size as multiples of REAL 
};

// between each pair of shared factors there is a Lagrangean multipliers. Does not scale. (quadratic number of multipliers)
class Lagrangean_factor_quadratic : public Lagrangean_factor_base {
public:
  using Lagrangean_factor_base::Lagrangean_factor_base;

  static INDEX joint_no_Lagrangean_vars(std::vector<Lagrangean_factor_quadratic>& factors)
  {
    const INDEX n = factors.size();
    assert(n > 0);
    const INDEX no_Lagrangean_vars = factors[0].no_Lagrangean_vars_;
    return (n*(n-1))/2 * no_Lagrangean_vars; 
  }

  static void init_Lagrangean_variables(std::vector<Lagrangean_factor_quadratic>& factors, const INDEX Lagrangean_vars_begin)
  {
    for(INDEX i=0; i<factors.size(); ++i) {
      factors[i].no_trees = factors.size();
      factors[i].pos = i;
      factors[i].global_Lagrangean_vars_offset_ = Lagrangean_vars_begin;
    } 
  }

  void add_to_mapping(std::vector<int>& mapping)
  {
    assert(false);
  }

  void serialize_Lagrangean(const double* wi, const double scaling)
  {
    for(INDEX i=0; i<pos; ++i) {
      const double* w = wi+offset(i);
      serialization_archive ar(w, no_Lagrangean_vars_*sizeof(REAL));
      addition_archive l_ar(ar, 1.0*scaling);
      f->serialize_dual(l_ar);
      ar.release_memory();
    }
    for(INDEX i=pos+1; i<no_trees; ++i) {
      //double* w = wi+offset(pos, i);
      const double* w = wi+offset(i-1);
      serialization_archive ar(w, no_Lagrangean_vars_*sizeof(REAL));
      addition_archive l_ar(ar, -1.0*scaling);
      f->serialize_dual(l_ar);
      ar.release_memory();
    }
  } 

  void copy_fn(double* wi)
  {
    for(INDEX i=0; i<pos; ++i) {
      double* w = wi+offset(i);
      f->subgradient(w, +1.0);
    } 
    for(INDEX i=pos+1; i<no_trees; ++i) {
      //double* w = wi+offset(pos,i);
      double* w = wi+offset(i-1);
      f->subgradient(w, -1.0);
    }
  }

  REAL dot_product_fn(double* wi)
  {
    REAL d = 0.0;
    for(INDEX i=0; i<pos; ++i) {
      double* w = wi+offset(i);
      d += f->dot_product(w);
    } 
    for(INDEX i=pos+1; i<no_trees; ++i) {
      double* w = wi+offset(i-1);
      d -= f->dot_product(w);
    }
    return d;
  }


protected:
  INDEX no_trees; // number of trees in which factor is
  INDEX pos; // factor is in pos-th tree that contains it

  INDEX global_offset(const INDEX offset, const INDEX i, const INDEX j) // offset for Lagrangean variables (i,j)
  {
    //assert(no_trees <= 2);
    assert(i < j && j < no_trees);
    assert(offset == global_Lagrangean_vars_offset_);
    assert(false); // remove offset parameter
    return offset + (i*no_trees - i*(i+1)/2 + (j-i-1))*no_Lagrangean_vars_; 
  }

  INDEX offset(const INDEX i) // offset for Lagrangean variables (i,j)
  {
    assert(i < no_trees);
    const INDEX o = local_Lagrangean_vars_offset_ + i*no_Lagrangean_vars_;
    return o;
  }
};

// extends factor_tree by collection of Lagrangean factors
template<typename FMC, typename LAGRANGEAN_FACTOR>
class LP_tree_Lagrangean : public factor_tree<FMC> {
public:
  LP_tree_Lagrangean(factor_tree<FMC>& t)
    : factor_tree<FMC>(t)
  {}

   template<typename VECTOR1>
   void compute_mapped_subgradient(VECTOR1& subgradient)
   {
      //const REAL subgradient_value = compute_subgradient();
      //assert(false); // assert that subgradient has been computed!
      std::vector<double> local_subgradient(mapping_.size(),0.0);
      // write primal solution into subgradient
      for(auto L : Lagrangean_factors_) {
         L.copy_fn(&local_subgradient[0]);
      }
      assert(mapping_.size() >= dual_size());
      for(INDEX i=0; i<mapping_.size(); ++i) {
         assert(mapping_[i] < subgradient.size());
         subgradient[ mapping_[i] ] += local_subgradient[i];
      } 
   }

   template<typename VECTOR>
   void compute_subgradient(VECTOR& subgradient, const REAL step_size)
   {
      //const REAL subgradient_value = compute_subgradient();
     assert(false); // assert that subgradient has been computed!

      std::fill(subgradient.begin(), subgradient.end(), 0.0);
      // write primal solution into subgradient
      for(auto L : Lagrangean_factors_) {
         L.copy_fn(&subgradient[0]);
      }
   }

  
   // find out necessary size for storing primal solution in archive
   INDEX compute_primal_size_in_bytes()
   {
      // why not allocate_archive?
      INDEX size = 0;
      for(auto& L : Lagrangean_factors_) {
         size += L.f->primal_size_in_bytes();
      }
      return size;
   }

   INDEX primal_size_in_bytes()
   { 
      assert(primal_size_in_bytes_ == compute_primal_size_in_bytes());
      return primal_size_in_bytes_; 
   }
   

   void add_weights(const double* wi, const double scaling)
   {
      for(auto& L : Lagrangean_factors_) {
         L.serialize_Lagrangean(wi, scaling);
      }
   }

  // dual size of Lagrangeans connected to current tree
  INDEX compute_dual_size_in_bytes()
  {
     INDEX s = 0;
     for(auto& L : Lagrangean_factors_) {
        s += L.no_Lagrangean_vars()*sizeof(REAL);
     }
     return s;
  }

  INDEX dual_size_in_bytes()
  { 
     assert(dual_size_in_bytes_ == compute_dual_size_in_bytes());
     assert(dual_size_in_bytes_ % sizeof(REAL) == 0);
     return dual_size_in_bytes_; 
  }

  INDEX dual_size() { return dual_size_in_bytes() / sizeof(REAL); }
   
  void read_in_primal(void* p)
  {
    serialization_archive ar(p, this->primal_size_in_bytes());
    load_archive l_ar(ar);
    for(auto L : Lagrangean_factors_) {
      L.f->serialize_primal(l_ar);
    } 
    ar.release_memory();
  }

  void save_primal(void* p)
  {
    serialization_archive ar(p, this->primal_size_in_bytes());
    save_archive s_ar(ar);
    for(auto L : Lagrangean_factors_) {
      L.f->serialize_primal(s_ar);
    } 
    ar.release_memory();
  }

  void init()
  {
    factor_tree<FMC>::init();
    dual_size_in_bytes_ = compute_dual_size_in_bytes();
    primal_size_in_bytes_ = compute_primal_size_in_bytes();
  }

//protected:
   INDEX primal_size_in_bytes_;
   INDEX dual_size_in_bytes_;
   // subgradient information = primal solution to tree
   // for sending messages down we need to know to how many lower factors an upper factor is connected and then we need to average messages sent down appropriately.

   // Lagrangean variables come from copying factors across trees
   std::vector<LAGRANGEAN_FACTOR> Lagrangean_factors_;

   const std::vector<int>& mapping() const { return mapping_; }
   std::vector<int>& mapping() { return mapping_; }

   // we have Lagrangean variables for every pair of factors that are identical but occur in different trees, hence can be indexed by factor and their pos
   // Lagrangean variables are stored contiguosly for each factor in lexicographic order of pos

   INDEX subgradient_size;
   std::vector<int> mapping_;

   std::vector<FactorTypeAdapter*> original_factors_;
};

// do zrobienia: templatize base class
template<typename FMC, typename LAGRANGEAN_FACTOR, typename DECOMPOSITION_SOLVER>
class LP_with_trees : public LP<FMC>
{
public:
   LP_with_trees(TCLAP::CmdLine& cmd)
     : LP<FMC>(cmd),
     tree_decomposition_begin_arg_("","treeDecompositionBegin","after how many iterations to start tree decomposition based optimization", false, 0, "", cmd)
  {}

   ~LP_with_trees()
   {
     // redirect messages back to original factors
     for(auto& t : trees_) {
       assert(t.original_factors_.size() == t.Lagrangean_factors_.size());
       std::unordered_map<FactorTypeAdapter*, FactorTypeAdapter*> copy_to_original_factor;
       for(INDEX i=0; i<t.Lagrangean_factors_.size(); ++i) {
         copy_to_original_factor.insert({t.Lagrangean_factors_[i].f, t.original_factors_[i]});
       } 

       for(auto tree_msg : t.tree_messages_) {
         auto* m = std::get<0>(tree_msg);
         auto* left = m->GetLeftFactor();
         auto* right = m->GetRightFactor();
         if(copy_to_original_factor.find(left) != copy_to_original_factor.end()) {
           auto* left_original = copy_to_original_factor.find(left)->second;
           m->SetLeftFactor(left_original);
         }
         if(copy_to_original_factor.find(right) != copy_to_original_factor.end()) {
           auto* right_original = copy_to_original_factor.find(right)->second;
           m->SetRightFactor(right_original);
         }
       } 
     }

     // delete copies of factors
     for(auto& t : trees_) {
       for(auto& L : t.Lagrangean_factors_) {
         delete L.f;
       }
     }
   }

   void add_tree(factor_tree<FMC>& t)
   { 
     LP_tree_Lagrangean<FMC,LAGRANGEAN_FACTOR> lt(t);
     trees_.push_back(lt);
   }

   // find out, which factors are shared between trees and add Lagrangean multipliers for them.
   void construct_decomposition()
   {
     constructed_decomposition = true;
      // first, go over all Lagrangean factors in each tree and count how often factor is shared
      struct Lagrangean_counting {
         //INDEX position = 0; // counter for enumerating in which position (i.e. in how many trees was factor already observed)
         std::vector<INDEX> trees;
         std::vector<LAGRANGEAN_FACTOR> factors;
         INDEX offset = 0; // offset into dual weights
      };
      std::unordered_map<FactorTypeAdapter*, Lagrangean_counting > Lagrangean_factors;
      for(INDEX i=0; i<trees_.size(); ++i) {
         for(auto* f : trees_[i].factors_) {
            auto it = Lagrangean_factors.find(f);
            if(it == Lagrangean_factors.end()) {
               it = Lagrangean_factors.insert({f,Lagrangean_counting()}).first;
            }
            it->second.trees.push_back(i);
         }
      }
      assert(Lagrangean_factors.size() == this->f_.size()); // otherwise not all factors are covered by trees

      // copy Lagrangean factors and insert into trees.
      Lagrangean_vars_size_ = 0;
      std::vector<std::unordered_map<FactorTypeAdapter*, FactorTypeAdapter*>> factor_mapping(trees_.size()); // original to copied factor in each tree
      for(auto& it : Lagrangean_factors) {
        auto* f = it.first;
        auto L = it.second;
        auto& tree_indices = L.trees;
        const INDEX no_occurences = tree_indices.size();
        if(no_occurences > 1) {
          f->divide(no_occurences);
          //L.factors.push_back(LAGRANGEAN_FACTOR(f));

          for(INDEX i : tree_indices) {
            auto* f_copy = f->clone(); // do zrobienia: possibly not all pointers to messages have to be cloned as well
            L.factors.push_back(LAGRANGEAN_FACTOR(f_copy));
            factor_mapping[i].insert(std::make_pair(f, f_copy)); 
          }

          const INDEX no_Lagrangean_vars = LAGRANGEAN_FACTOR::joint_no_Lagrangean_vars( L.factors );
          LAGRANGEAN_FACTOR::init_Lagrangean_variables( L.factors, Lagrangean_vars_size_ );

          assert(tree_indices.size() == L.factors.size());
          for(INDEX i=0; i<L.factors.size(); ++i) {
            const INDEX tree_index = tree_indices[i];
            trees_[tree_index].Lagrangean_factors_.push_back(L.factors[i]);
            trees_[tree_index].original_factors_.push_back(f);
          }
          
          std::cout << no_Lagrangean_vars << "; " << Lagrangean_vars_size_ << "\n";
          Lagrangean_vars_size_ += no_Lagrangean_vars;
        }
      }

      // Redirect links to factors in relevant messages
      for(INDEX i=0; i<trees_.size(); ++i) {
         auto& t = trees_[i];
         
         // redirect links from messages in trees that are directed to current factor
         for(auto tree_msg : trees_[i].tree_messages_) {
            auto* m = std::get<0>(tree_msg);
            auto* left = m->GetLeftFactor();
            auto* right = m->GetRightFactor();
            if(factor_mapping[i].find(left) != factor_mapping[i].end()) {
               auto* left_copy = factor_mapping[i].find(left)->second;
               m->SetLeftFactor(left_copy);
            }
            if(factor_mapping[i].find(right) != factor_mapping[i].end()) {
               auto* right_copy = factor_mapping[i].find(right)->second;
               m->SetRightFactor(right_copy);
            }
         }
         // search for factor in tree and change it as well
         for(auto& f : t.factors_) {
            if(factor_mapping[i].find(f) != factor_mapping[i].end()) {
               auto* f_copy = factor_mapping[i].find(f)->second;
               f = f_copy;
            } 
         }
      }

      // set primal and dual size for tree
      for(auto& t : trees_) {
         t.init();
      }

      // construct mapping from Lagrangean variables of each tree to whole Lagrangean variables
      //mapping_.reserve(trees_.size());
      for(auto& t : trees_) {
         INDEX local_offset = 0;
         std::vector<int> m;
         m.reserve(t.dual_size());
         for(auto& L : t.Lagrangean_factors_) {
           L.add_to_mapping(m);
         }
         assert(m.size() <= Lagrangean_vars_size_);
         //assert(m.size() == t.dual_size());
         t.mapping_ = m;
      }

      // check map validity: each entry in m (except last one) must occur exactly twice
      assert(mapping_valid()); 

      static_cast<DECOMPOSITION_SOLVER*>(this)->construct_decomposition();
   }

   bool mapping_valid() const
   {
      std::vector<INDEX> mapping_count(Lagrangean_vars_size_,0);
      for(const auto& t : trees_) {
         for(INDEX i=0; i<t.mapping().size(); ++i) {
            mapping_count[t.mapping()[i]]++;
         }
      }
      for(auto i : mapping_count) {
         if(i < 2 || i > trees_.size()) {
           assert(false);
            return false;
         }
      }
      return true;
   }

   INDEX no_Lagrangean_vars() const { return Lagrangean_vars_size_; } 
   
   void ComputeForwardPassAndPrimal(const INDEX iteration) 
   {
      assert(false);
   }

   void ComputeBackwardPassAndPrimal(const INDEX iteration) 
   {
      assert(false); 
   }

   virtual void ComputePass(const INDEX iteration)
   {
     const INDEX tree_decomposition_iter = tree_decomposition_begin_arg_.getValue();
     if(iteration < tree_decomposition_iter) {
       LP<FMC>::ComputePass(iteration);
     } else if(iteration == tree_decomposition_iter) {
       construct_decomposition();
       static_cast<DECOMPOSITION_SOLVER*>(this)->optimize_decomposition(iteration); 
     } else {
       static_cast<DECOMPOSITION_SOLVER*>(this)->optimize_decomposition(iteration); 
     }
   }

   // lower bound must be computed over all cloned factors as well
   REAL LowerBound() 
   {
     if(constructed_decomposition) {
       return static_cast<DECOMPOSITION_SOLVER*>(this)->decomposition_lower_bound();
     } else {
       return LP<FMC>::LowerBound();
     }
   }

   REAL decomposition_lower_bound() const
   {
     REAL lb = 0.0;
     for(auto& t : trees_) {
       lb += t.lower_bound();
     }
     return lb; 
   }

   void add_weights(const double* w, const REAL scaling) 
   {
      for(INDEX i=0; i<trees_.size(); ++i) {
         auto& tree = trees_[i];
         const auto& m = tree.mapping();
         std::vector<double> local_weights;
         local_weights.reserve(m.size());
         for(INDEX idx=0; idx<m.size(); ++idx) {
            local_weights.push_back(w[m[idx]]);
         }
         tree.add_weights(&local_weights[0], scaling);
      }
   }


protected:
   std::vector<LP_tree_Lagrangean<FMC,LAGRANGEAN_FACTOR>> trees_; // store for each tree the associated Lagrangean factors.
   INDEX Lagrangean_vars_size_;
   TCLAP::ValueArg<INDEX> tree_decomposition_begin_arg_; 
   bool constructed_decomposition = false;
};

// perform subgradient ascent with Polyak's step size with estimated optimum
template<typename FMC>
class LP_subgradient_ascent : public LP_with_trees<FMC, Lagrangean_factor_quadratic, LP_subgradient_ascent<FMC>> // better: perform projected subgradient ascent with Lagrangean_factor_zero_sum
{
public:
   using LP_with_trees<FMC, Lagrangean_factor_quadratic, LP_subgradient_ascent<FMC>>::LP_with_trees;

   void construct_decomposition() {}

   void optimize_decomposition(const INDEX iteration)
   {
      REAL current_lower_bound = 0.0;
      std::vector<REAL> subgradient(this->no_Lagrangean_vars(), 0.0);
      for(INDEX i=0; i<this->trees_.size(); ++i) {
         current_lower_bound += this->trees_[i].solve();
         this->trees_[i].compute_mapped_subgradient(subgradient); // note that mapping has one extra component!
         //trees_[i].primal_cost();
      }
      best_lower_bound = std::max(current_lower_bound, best_lower_bound);
      assert(std::find_if(subgradient.begin(), subgradient.end(), [](auto x) { return x != 0.0 && x != 1.0 && x != -1.0; }) == subgradient.end());
      const REAL subgradient_one_norm = std::accumulate(subgradient.begin(), subgradient.end(), 0.0, [=](REAL s, REAL x) { return s + std::abs(x); });

      const REAL step_size = (best_lower_bound - current_lower_bound + subgradient.size())/(10.0 + iteration) / subgradient_one_norm;

      std::cout << "stepsize = " << step_size << ", absolute value of subgradient = " << subgradient_one_norm << "\n";
      this->add_weights(&subgradient[0], step_size);
   }

private:
   REAL best_lower_bound;
   REAL step_size;
};

} // end namespace LP_MP

#endif // LP_MP_TREE_DECOMPOSITION_HXX

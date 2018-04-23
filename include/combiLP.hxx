#ifndef LP_MP_combiLP_HXX
#define LP_MP_combiLP_HXX

#include <iostream>
#include <unordered_set>
#include <vector>

#include "LP_MP.h"
#include "partial_external_solver.hxx"
#include "factor_archive.hxx"

namespace LP_MP {

template<typename EXTERNAL_SOLVER, typename BASE_LP>
class combiLP : public BASE_LP {
public:
  using BASE_LP::BASE_LP;

  double LowerBound() {
    return is_ilp_phase_ ? PseudoBound() : BASE_LP::LowerBound();
  }

  double PseudoBound() {
    double cost = this->constant_;
    for (auto* f : this->f_) {
      assert(f->LowerBound() <= f->EvaluatePrimal() + eps);
      cost += f->EvaluatePrimal();
    }
    return cost;
  }

  void End() {
    is_ilp_phase_ = true;

    enum class State { LP, Active, ILP };

#ifndef NDEBUG
    auto state_to_string = [](State s) {
      switch(s) {
        case State::LP:
          return "State::LP";
        case State::Active:
          return "State::Active";
        case State::ILP:
          return "State::ILP";
      }
    };
#endif

    INDEX size_lp, size_active, size_ilp;
    std::unordered_map<FactorTypeAdapter*, State> factor_states;
    partial_external_solver<EXTERNAL_SOLVER> external_solver;
    factor_archive<serialization_functor::primal> archive(this->f_.begin(), this->f_.end());
    double lower_bound = -std::numeric_limits<double>::infinity();
    double upper_bound = std::numeric_limits<double>::infinity();

#ifndef NDEBUG
    auto check_invariant = [&]() {
      // The primal assignment in LP region must not have been modified. Note
      // that the Active regions theoretically belongs to the LP part, but we
      // allow modificatons as the algorithm works actively on it and performs
      // optimality checking by modifying the assignment and checking the
      // bounds.
      decltype(archive) a(this->f_.begin(), this->f_.end());
      this->for_each_factor([&](auto* f) {
        assert(factor_states.find(f) != factor_states.end());
        if (factor_states[f] == State::LP) {
          assert(decltype(archive)::check_factor_equality(archive, a, f));
        }
      });

      // Messages inside LP and ILP have to be consistent (messages on borders
      // are excluded).
      this->for_each_message([&](auto* m) {
        auto* l = m->GetLeftFactor(); bool l_in_ilp = external_solver.has_factor(l);
        auto* r = m->GetRightFactor(); bool r_in_ilp = external_solver.has_factor(r);
        if (l_in_ilp == r_in_ilp)
          assert(m->CheckPrimalConsistency());
      });
    };
#endif

    auto update_partition = [&]() {
      this->for_each_factor([&](auto* f) {
        assert(f->LowerBound() <= f->EvaluatePrimal() + eps);
        assert(factor_states.find(f) != factor_states.end());
        switch (factor_states[f]) {
        case State::LP:
          archive.load_factor(f);
          break;
        case State::Active:
          if (f->LowerBound() < f->EvaluatePrimal() - eps) // not locally optimal
            external_solver.add_factor(f);
          break;
        case State::ILP:
          break;
        };
      });

      this->for_each_message([&](auto* m) {
        if (!m->CheckPrimalConsistency()) { // no factor agreement
#ifndef NDEBUG
          bool handled = false;
#endif
          std::array<FactorTypeAdapter*, 2> tmp = { m->GetLeftFactor(), m->GetRightFactor() };
          for (auto* f : tmp) {
            assert(factor_states.find(f) != factor_states.end());
            if (factor_states[f] == State::Active) {
              external_solver.add_factor(f);
#ifndef NDEBUG
              handled = true;
#endif
            }
          }
          assert(handled);
        }
      });

      size_lp = 0; size_active = 0; size_ilp = 0;
      for (auto* f : this->f_) {
        assert(factor_states.find(f) != factor_states.end());
        if (external_solver.has_factor(f)) {
          factor_states[f] = State::ILP;
          ++size_ilp;
        } else {
          factor_states[f] = State::LP;
          ++size_lp;
        }
      }
      assert(size_lp + size_active + size_ilp == this->f_.size());

      this->for_each_message([&](auto* m) {
        auto* lf = m->GetLeftFactor();
        auto* rf = m->GetRightFactor();
        assert(factor_states.find(lf) != factor_states.end());
        assert(factor_states.find(rf) != factor_states.end());
        auto lfs = factor_states[lf];
        auto rfs = factor_states[rf];
        if (lfs == State::LP && rfs == State::ILP) {
          factor_states[lf] = State::Active;
          --size_lp;
          ++size_active;
        } else if (lfs == State::ILP && rfs == State::LP) {
          factor_states[rf] = State::Active;
          --size_lp;
          ++size_active;
        }
      });
      assert(size_lp + size_active + size_ilp == this->f_.size());
    };

    // Initialize first ILP subproblem
    for (auto* f : this->f_)
      factor_states[f] = State::Active;
    update_partition();

    // Iterate until convergence (dirty flag basically signals consistency).
    int iteration = 0;
    while (external_solver.dirty()) {
#ifndef NDEBUG
      check_invariant();
#endif

#ifndef LP_MP_COMBILP_DISABLE_BRIDGE_FACTOR_OPTIMIZATION
      // Optional Optimization: Add neighbors of "bridging" factors (at most 2
      // neighbors). This corresponds to "pairwise" edges for original CombiLP
      // on Graphical Models. This should be a huge performance boost as it
      // reduces the number of iterations.
      INDEX bridge_count = external_solver.GetNumberOfFactors();
      this->for_each_factor([&](auto* f) {
        assert(factor_states.find(f) != factor_states.end());
        if (factor_states[f] == State::Active && f->no_messages() <= 2) { // is "active" bridging factor
          for (auto& msg_it : f->get_messages()) {
            assert(factor_states.find(msg_it.adjacent_factor) != factor_states.end());
            auto& neighboring_state = factor_states[msg_it.adjacent_factor];
            if (neighboring_state == State::LP)
              neighboring_state = State::Active;
          }
        }
      });
      bridge_count = external_solver.GetNumberOfFactors() - bridge_count;
      std::cout << "CombiLP: Added " << bridge_count << " bridging factors." << std::endl;
#endif // LP_MP_COMBILP_DISABLE_BRIDGE_FACTOR_OPTIMIZATION

      // Reparametrize border: Improves convergence.
      // TODO: Evaluate if this is really necessary and improves convergence
      // significantly.
      this->for_each_message([&](auto* m) {
        auto* l = m->GetLeftFactor();
        auto* r = m->GetRightFactor();
        if (external_solver.has_factor(l) && !external_solver.has_factor(r))
          m->send_message_to_left();
        if (!external_solver.has_factor(l) && external_solver.has_factor(r))
          m->send_message_to_right();
      });

      // Add messages connecting all factors in the ILP.
      external_solver.add_messages(*this);

      ++iteration;
      std::cout << std::endl << "CombiLP iteration " << iteration << ": "
                << "lp=" << size_lp << " "
                << "active=" << size_active << " "
                << "ilp=" << size_ilp << " / "
                << this->GetNumberOfFactors() << " ("
                << (100.0f * size_ilp / this->GetNumberOfFactors())
                << "%)" << std::endl;

      const bool solved = external_solver.solve();
      assert(solved);

#ifndef NDEBUG
      check_invariant();
#endif

      lower_bound = this->LowerBound();

      // We now propagate the primal assignment of the boundary regions of the
      // ILP to the LP. We do it for all ILP nodes, as the ILP part is already
      // consistent so nothing will happen there (it is unnecessary work
      // though).
      //
      // Note that this will change the LP part (not only the directly
      // neighboring factors as factors might have "upstream" factors, see e.g.
      // `UnarySimplexFactor`). This will be fixed in `update_partition` (LP
      // part gets restored, "active" part of LP remains modified, as
      // optimality is checked by comparing bounds).
      for (auto* f : this->f_)
        if (external_solver.has_factor(f))
          f->propagate_primal_through_messages();

      upper_bound = this->EvaluatePrimal();
      assert(lower_bound <= upper_bound + eps);

      std::cout << "CombiLP: lower bound = " << lower_bound << " / "
                << "upper bound = " << upper_bound << " (diff = "
                << upper_bound - lower_bound << ")" << std::endl;

      // Does the real science: Restores LP primal assignment, checks boundary
      // consistency on "active" part of the LP. Hence implements optimality
      // check. :)
      update_partition();
#ifndef NDEBUG
      check_invariant();
      if (std::abs(upper_bound - lower_bound) > eps)
        assert(external_solver.dirty());
#endif
    }

#ifndef NDEBUG
    lower_bound = this->LowerBound();
    upper_bound = this->EvaluatePrimal();

    this->for_each_message([&](auto* m) { assert(m->CheckPrimalConsistency()); });
    assert(std::abs(upper_bound - lower_bound) <= eps);

    // Check the invariant one last time. As we have solved the problem there
    // is no "Active" region anymore. The primal assignment in the "Active"
    // region might still have been modified, as only the bounds have been
    // checked. Additionally to the normal `check_invariant` we just make sure
    // that the LP+Active region is really locally optimal.
    check_invariant();
    for (auto* f : this->f_) {
      assert(factor_states.find(f) != factor_states.end());
      if (factor_states[f] != State::ILP)
        assert(std::abs(f->LowerBound() - f->EvaluatePrimal()) <= eps);
    }
#endif
  }

private:
  bool is_ilp_phase_ = false;
};

} // namespace LP_MP

#endif // LP_MP_combiLP_HXX

// vim: set ts=2 sts=2 sw=2 et fdm=marker fmr=#if,#endif:

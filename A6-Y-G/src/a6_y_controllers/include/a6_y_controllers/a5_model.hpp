#pragma once
#include <pinocchio/parsers/urdf.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace a6_y_controllers {
inline void load_mimic_model(const std::string& path, pinocchio::Model& model) {
  // Pinocchio appends into an existing model. Lifecycle reconfiguration must
  // reload a fresh tree instead of doubling its joints/coordinates.
  model = pinocchio::Model();
  pinocchio::urdf::buildModel(path, model, false, true);
}
inline void map_independent_joints(const pinocchio::Model& model,
    const std::vector<std::string>& names, std::vector<int>& q, std::vector<int>& v) {
  if (model.nq != static_cast<int>(names.size()) || model.nv != static_cast<int>(names.size()))
    throw std::runtime_error("All independent model joints must have measured state");
  q.clear(); v.clear();
  for (const auto& name : names) {
    if (!model.existJointName(name)) throw std::runtime_error("Missing joint: " + name);
    const auto id = model.getJointId(name);
    if (model.nqs[id] != 1 || model.nvs[id] != 1)
      throw std::runtime_error("Expected one independent coordinate: " + name);
    q.push_back(model.idx_qs[id]); v.push_back(model.idx_vs[id]);
  }
  if (std::set<int>(q.begin(), q.end()).size() != names.size() ||
      std::set<int>(v.begin(), v.end()).size() != names.size())
    throw std::runtime_error("Duplicate independent joint mapping");
}
}  // namespace a6_y_controllers

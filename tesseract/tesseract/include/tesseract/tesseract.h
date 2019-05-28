/**
 * @file tesseract.h
 * @brief This is a container class for all tesseract packages. Provides
 * methods to simplify construction of commonly used features.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_TESSERACT_H
#define TESSERACT_TESSERACT_H

#include <tesseract_environment/core/environment.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <memory>
#include <boost/filesystem.hpp>

namespace tesseract
{
/**
 * @brief The Tesseract class
 *
 * This is a container class which hold objects needed for motion planning.
 * It also provides several construction methods for loading from urdf, srdf
 *
 */
class Tesseract
{
public:
  using Ptr = std::shared_ptr<Tesseract>;
  using ConstPtr = std::shared_ptr<const Tesseract>;

  Tesseract();
  virtual ~Tesseract() = default;

  bool isInitialized() const;

  bool init(tesseract_scene_graph::SceneGraphPtr scene_graph);
  bool init(tesseract_scene_graph::SceneGraphPtr scene_graph, tesseract_scene_graph::SRDFModelConstPtr srdf_model);
  bool init(const std::string& urdf_string, tesseract_scene_graph::ResourceLocatorFn locator);
  bool init(const std::string& urdf_string, const std::string& srdf_string, tesseract_scene_graph::ResourceLocatorFn locator);
  bool init(const boost::filesystem::path& urdf_path, tesseract_scene_graph::ResourceLocatorFn locator);
  bool init(const boost::filesystem::path& urdf_path, const boost::filesystem::path& srdf_path, tesseract_scene_graph::ResourceLocatorFn locator);

  const tesseract_environment::EnvironmentPtr& getEnvironment();
  const tesseract_environment::EnvironmentConstPtr& getEnvironmentConst() const;
  const tesseract_scene_graph::SRDFModelConstPtr& getSRDFModel() const;
  const tesseract_kinematics::ForwardKinematicsConstPtrMap& getFwdKinematics();
  tesseract_kinematics::ForwardKinematicsConstPtr getFwdKinematics(const std::string& name) const;

  // TODO: Add inverse kinematics

private:
  bool initialized_;
  tesseract_environment::EnvironmentPtr environment_;
  tesseract_environment::EnvironmentConstPtr environment_const_;
  tesseract_scene_graph::SRDFModelConstPtr srdf_model_;
  tesseract_kinematics::ForwardKinematicsConstPtrMap fwd_kin_map_;
//  const tesseract_kinematics::InverseKinematicsConstPtrMap inv_kin_map_;

  bool registerDefaultContactManagers();

  void clear();
};

}
#endif // TESSERACT_TESSERACT_H

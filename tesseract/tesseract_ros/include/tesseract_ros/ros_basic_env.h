/**
 * @file basic_env.h
 * @brief Basic low-level environment with collision and distance functions.
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
#ifndef ROS_BASIC_ENV_H
#define ROS_BASIC_ENV_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <geometric_shapes/shape_operations.h>
#include <tesseract_core/basic_env.h>
#include <map>
#include <unordered_map>

namespace tesseract
{

struct ROSAllowedCollisionMatrix : public AllowedCollisionMatrix
{
  /**
   * @brief Disable collision between two collision objects
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   * @param reason The reason for disabling collison
   */
  virtual void addAllowedCollision(const std::string &link_name1, const std::string &link_name2, const std::string &reason)
  {
    lookup_table_[link_name1 + link_name2] = reason;
    lookup_table_[link_name2 + link_name1] = reason;
  }

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   */
  virtual void removeAllowedCollision(const std::string &link_name1, const std::string &link_name2)
  {
    lookup_table_.erase(link_name1 + link_name2);
    lookup_table_.erase(link_name2 + link_name1);
  }

  bool isCollisionAllowed(const std::string &link_name1, const std::string &link_name2) const
  {
    return (lookup_table_.find(link_name1 + link_name2) != lookup_table_.end());
  }

private:
  std::unordered_map<std::string, std::string> lookup_table_;
};
typedef boost::shared_ptr<ROSAllowedCollisionMatrix> ROSAllowedCollisionMatrixPtr;
typedef boost::shared_ptr<const ROSAllowedCollisionMatrix> ROSAllowedCollisionMatrixConstPtr;

struct AttachedBodyInfo
{
  std::string name;
  std::string parent_link_name;
  std::string object_name;
  std::vector<std::string> touch_links;
};

struct AttachableObject
{
  std::string name;
  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Affine3d shapes_trans;
};
typedef boost::shared_ptr<AttachableObject> AttachableObjectPtr;
typedef boost::shared_ptr<const AttachableObject> AttachableObjectConstPtr;

struct AttachedBody
{
   AttachedBodyInfo info;
   AttachableObjectConstPtr obj;
};
typedef boost::shared_ptr<AttachedBody> AttachedBodyPtr;
typedef boost::shared_ptr<const AttachedBody> AttachedBodyConstPtr;

class ROSBasicEnv : public BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ROSBasicEnv() : allowed_collision_matrix_(new ROSAllowedCollisionMatrix()) {}

  /**
   * @brief A a manipulator as a kinematic chain
   * @param base_link The base link of the chain
   * @param tip_link The tip link of the chain
   * @param name The name of the manipulator. This must be unique.
   * @return true if successfully created, otherwise false.
   */
  virtual bool addManipulator(const std::string &base_link, const std::string &tip_link, const std::string &manipulator_name) = 0;

  /**
   * @brief Add object so it may be attached/detached.
   *
   * This object is not part of the environment until attached to a link.
   *
   * @param attachable_object The object information
   */
  virtual void addAttachableObject(const AttachableObjectConstPtr &attachable_object) = 0;

  /**
   * @brief Get object attached to the manipulator or world
   * @param name The name of the object
   * @return AttachedBody
   */
  virtual const AttachedBodyConstPtr getAttachedBody(const std::string& name) const = 0;

  /**
   * @brief Attached an attachable object to the environment
   * @param attached_body Information of attaching creating the attached body
   */
  virtual void attachBody(const AttachedBodyInfo &attached_body_info) = 0;

  /**
   * @brief Detach an attachable object from the environment
   * @param name The name given to the Attached Body when attached
   */
  virtual void detachBody(const std::string &name) = 0;

  /////////////////////////
  // Implemented Methods //
  /////////////////////////

  AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const
  {
    return allowed_collision_matrix_;
  }

  /** @brief Get the allowed collision matrix non const */
  virtual ROSAllowedCollisionMatrixPtr getAllowedCollisionMatrixNonConst() const
  {
    return allowed_collision_matrix_;
  }

protected:
  ROSAllowedCollisionMatrixPtr allowed_collision_matrix_; /**< The allowed collision matrix used during collision checking */

}; // class ROSBasicEnv

typedef boost::shared_ptr<ROSBasicEnv> ROSBasicEnvPtr;
typedef boost::shared_ptr<const ROSBasicEnv> ROSBasicEnvConstPtr;

} //namespace tesseract

#endif // ROS_BASIC_ENV_H

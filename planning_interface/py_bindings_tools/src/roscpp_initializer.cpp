/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
/* Auther: Isao Hara */
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <memory>
#include <thread>
#include <mutex>

static std::vector<std::string>& ROScppArgs()
{
  static std::vector<std::string> args;
  return args;
}

static std::string& ROScppNodeName()
{
  static std::string node_name("moveit_python_wrappers");
  return node_name;
}

void moveit::py_bindings_tools::roscpp_set_arguments(const std::string& node_name, boost::python::list& argv)
{
  ROScppNodeName() = node_name;
  ROScppArgs() = stringFromList(argv);
}

namespace
{
struct InitProxy
{
  InitProxy()
  {
    const std::vector<std::string>& args = ROScppArgs();
    int fake_argc = args.size();
    char** fake_argv = new char*[args.size()];
    for (std::size_t i = 0; i < args.size(); ++i){
      fake_argv[i] = strdup(args[i].c_str());
    }
    if (fake_argc == 0){
      rclcpp::init(fake_argc, NULL);
    }else{
      rclcpp::init(fake_argc, fake_argv);
    }
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared(ROScppNodeName(), node_options);
    for (int i = 0; i < fake_argc; ++i)
      delete[] fake_argv[i];
    delete[] fake_argv;

  }

  ~InitProxy()
  {
    if (rclcpp::ok()){
      rclcpp::shutdown();
    }
  }

  std::shared_ptr<rclcpp::Node> getNode()
  {
    return node;
  }

  std::shared_ptr<rclcpp::Node> node;
};
}  // namespace

//static void roscpp_init_or_stop(bool init)
static std::shared_ptr<rclcpp::Node> roscpp_init_or_stop(bool init)
{
  // ensure we do not accidentally initialize ROS multiple times per process
  static std::mutex lock;
  std::scoped_lock slock(lock);

  // once per process, we start a spinner
  static bool once = true;
  static std::unique_ptr<InitProxy> proxy;
  static std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

  // initialize only once
  if (once && init)
  {
    once = false;

    // if ROS (cpp) is not initialized, we initialize it
    if (!rclcpp::ok())
    {
      proxy = std::make_unique<InitProxy>();
#if 1 
      executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
      executor.get()->add_node(proxy.get()->getNode());
      std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, executor.get()));
      executor_thread.detach();
#endif
    }
  }

  // shutdown if needed
  if (!init)
  {
    fprintf(stderr, "... shutdown\n");
    once = false;
    proxy.reset();
    return nullptr;
  }
  return proxy.get()->getNode();
}

std::shared_ptr<rclcpp::Node> moveit::py_bindings_tools::get_rclcpp_node()
{
  static std::unique_ptr<InitProxy> proxy;
  return proxy->getNode();
}

std::shared_ptr<rclcpp::Node> moveit::py_bindings_tools::roscpp_init()
{
  return roscpp_init_or_stop(true);
}

void
moveit::py_bindings_tools::roscpp_init(const std::string& node_name, boost::python::list& argv)
{
  roscpp_set_arguments(node_name, argv);
  roscpp_init();
  return;
}

std::shared_ptr<rclcpp::Node>
moveit::py_bindings_tools::roscpp_init(boost::python::list& argv)
{
  ROScppArgs() = stringFromList(argv);
  return roscpp_init();
}

void moveit::py_bindings_tools::roscpp_shutdown()
{
  roscpp_init_or_stop(false);
}

moveit::py_bindings_tools::ROScppInitializer::ROScppInitializer()
{
  rclcpp_node = roscpp_init();
}

moveit::py_bindings_tools::ROScppInitializer::ROScppInitializer(boost::python::list& argv)
{
  rclcpp_node = roscpp_init(argv);
}

moveit::py_bindings_tools::ROScppInitializer::ROScppInitializer(const std::string& node_name, boost::python::list& argv)
{
  roscpp_set_arguments(node_name, argv);
  rclcpp_node = roscpp_init();
}

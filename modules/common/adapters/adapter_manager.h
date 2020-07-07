/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#ifndef MODULES_ADAPTERS_ADAPTER_MANAGER_H_
#define MODULES_ADAPTERS_ADAPTER_MANAGER_H_

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "modules/common/adapters/adapter.h"
#include "modules/common/adapters/message_adapters.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/log.h"
#include "modules/common/macro.h"

#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::common::adapter
 * @brief apollo::common::adapter
 */
namespace apollo {
namespace common {
namespace adapter {

/// Macro to prepare all the necessary adapter functions when adding a
/// new input/output. For example when you want to listen to
/// car_status message for your module, you can do
/// REGISTER_ADAPTER(CarStatus) write an adapter class called
/// CarStatusAdapter, and call EnableCarStatus(`car_status_topic`,
/// true, `callback`(if there's one)) in AdapterManager.
#define REGISTER_ADAPTER(name)                                                 \
 public:                                                                       \
  static void Enable##name(const std::string &topic_name,                      \
                           AdapterConfig::Mode mode,                           \
                           int message_history_limit) {                        \
    CHECK(message_history_limit > 0)                                           \
        << "Message history limit must be greater than 0";                     \
    instance()->InternalEnable##name(topic_name, mode, message_history_limit); \
  }                                                                            \
  /**                                                                          \
   * @brief InternalGet##name()得到指向ChassisAdapter=Adapter<canbus::Chassis>  \
   * 的普通指针，所以，GetChassis()这个函数可以得到ChassisAdapter的普通指针。          \
   */                                                                          \
  static name##Adapter *Get##name() {                                          \
    return instance()->InternalGet##name();                                    \
  }
  /**
   * @brief 这个现在看不懂，因为用了很多.proto文件生成类里的方法，都不知道是干啥的
   *  所以得对比着看，回家对着看吧。
   */
  static bool Feed##name##File(const std::string &proto_file) {                \
    if (!instance()->name##_) {                                                \
      AERROR << "Initialize adapter before feeding protobuf";                  \
      return false;                                                            \
    }                                                                          \
    return Get##name()->FeedFile(proto_file);                                  \
  }                                                                            \
  /**                                                                          \
   * @brief 这是提供给外面使用的Pub函数，调用了下面私有的pub函数                       \
   * 跟下面那个函数一样，我并不清楚 name##Adapter::DataType类型能不能顺利pub           \
   */                                                                          \
  static void Publish##name(const name##Adapter::DataType &data) {             \
    instance()->InternalPublish##name(data);                                   \
  }                                                                            \
  /**                                                                          \
   * @brief 填充时间戳的函数                                                      \
   *  static_assert()用来判断类型是不是相同，不相同的时候会输出打印信息，这个在编译阶段   \
   *  完成。然后调用Adapter中的FillHeader，我进去看了一下，它会调用proto编译成c++文件   \
   *  后的类的方法，往header里面填，但header是什么类型我还没深挖，这个得编译好proto后，   \
   *  到类里面去找mutable_header()函数的返回类型才行。                              \
   */                                                                          \
  template <typename T>                                                        \
  static void Fill##name##Header(const std::string &module_name, T *data) {    \
    static_assert(std::is_same<name##Adapter::DataType, T>::value,             \
                  "Data type must be the same with adapter's type!");          \
    instance()->name##_->FillHeader(module_name, data);                        \
  }                                                                            \
  /**                                                                          \
  * 下面这个就是真正的添加Callback了。以Chassis为例，name##Adapter::Callback =       \
  * ChassisAdapter::Callback = std::function<void(const canbus::Chassis&)>     \
  * 所以，这个callback()应该是返回void,入参为canbus::Chassis类型的参数。然后这个       \
  * callback 被传进Adapter的AddCallback()方法中。这个 AddCallback()我也进去看了，    \
  * 它把这个函数扔到std::vector<Callback>队列里，然后在FireCallbacks(const D& data) \
  * 里面遍历调用，FireCallbacks是在OnReceive(const D& message)里面调用的，这两个函数  \
  * 都是Adapter类里的方法哈。                                                     \
  */                                                                           \
  static void Add##name##Callback(name##Adapter::Callback callback) {          \
    CHECK(instance()->name##_)                                                 \
        << "Initialize adapter before setting callback";                       \
    instance()->name##_->AddCallback(callback);                                \
  }                                                                            \
  /* 下面这个挺有学问，fp是一个函数指针，这个指针在类T的作用域里，后面bind里面第一个参数*/  \
  /* 会是一个类的方法，那第二个参数就应该是这个类额指针，第三个占位符是方法的参数。注意到了*/ \
  /* 吗，函数体里还有一个Add##name##Callback，它是一个参数，对，它实际是上面的那个    */ \
  /* Add##name##Callback.*/                                                    \
  template <class T>                                                           \
  static void Add##name##Callback(                                             \
      void (T::*fp)(const name##Adapter::DataType &data), T *obj) {            \
    Add##name##Callback(std::bind(fp, obj, std::placeholders::_1));            \
  }                                                                            \
                                                                               \
 private:                                                                      \
  std::unique_ptr<name##Adapter> name##_;                                      \
  ros::Publisher name##publisher_;                                             \
  ros::Subscriber name##subscriber_;                                           \
  /*这个函数干的活挺多，定义了sub，明确了sub的回调函数，明确了pub的类型，还把Adapter里*/  \
  /*的Observe()推进vector里，用于后面的遍历调用。Observe()大概是用来更新各类型的消息的*/ \
  void InternalEnable##name(const std::string &topic_name,                     \
                            AdapterConfig::Mode mode,                          \
                            int message_history_limit) {                       \
  /* Chassis_.reset()是重置这个智能指针 */                                        \
    name##_.reset(                                                             \
        new name##Adapter(#name, topic_name, message_history_limit));          \
    if (mode != AdapterConfig::PUBLISH_ONLY && node_handle_) {                 \
  /* Chassissubscriber_就是个普通的ros::Subscriber对象 */                         \
  /* node_handle_是下面定义的 std::unique_ptr<ros::NodeHandle> */                \
      name##subscriber_ =                                                      \
          node_handle_->subscribe(topic_name, message_history_limit,           \
  /* 这个OnReceive是定义在Adapter里面的回调函数 */                                 \
                                  &name##Adapter::OnReceive, name##_.get());   \
    }                                                                          \
    if (mode != AdapterConfig::RECEIVE_ONLY && node_handle_) {                 \
  /* 这个publisher pub的是 Adapter<canbus::Chassis>::canbus::Chassis 类型消息 */  \
      name##publisher_ = node_handle_->advertise<name##Adapter::DataType>(     \ // name##Adapter::DataType = Adapter<canbus::Chassis>::canbus::Chassis
          topic_name, message_history_limit);                                  \   // name##Adapter = ChassisAdapter = Adapter<canbus::Chassis>
    }                                                                          \
  /* observers_ 是 std::vector<std::function<void()>> 类型，后面是把一个lamda函数*/       \
  /* 推到这个vector里面，而lamda里面又调用了Adapter<canbus::Chassis>类里面的Observe()函数*/  \
  /* 这应该是比较高级的用法，只是把后面的lamda表达是推进observers_,并不会执行函数体，然后*/       \
  /* 我猜会有个地方遍历执行它，怎么执行我也不知道，看到了再说吧。  */                            \
    observers_.push_back([this]() { name##_->Observe(); });                    \
  }                                                                            \
  /**                                                                          \ 
   * @brief 用来得到 指向Adapter<canbus::Chassis>的普通指针                        \
   * 会被上面的Get##name()调用                                                    \
   */                                                                          \
  name##Adapter *InternalGet##name() { return name##_.get(); }                 \
  /**                                                                          \
   * @brief pub 函数，上面的Publish##name会调用这个                                \
   * 这里我有个疑问，name##Adapter::DataType = ChassisAdapter::canbus::Chassis    \
   * 这样的类型也能用ros的publish吗？？？SetLatestPublished(data)这个是Adapter内     \
   * 的函数，我进去看了，它是更新std::unique_ptr<D> latest_published_data_ 这个      \
   * 成员变量的。更新完用在哪儿，我就没深挖了。                                        \
   */                                                                           \
  void InternalPublish##name(const name##Adapter::DataType &data) {            \ // InternalPublishChassis()这个函数，我理解是不会显式出现在其他模块里的，因为不是static的，
    /* Only publish ROS msg if node handle is initialized. */                  \         // 它在别人用AdapterManager::PublishChassis()时候被调用。
    if (node_handle_) {                                                        \         // 
      name##publisher_.publish(data);                                          \
    }                                                                          \
    name##_->SetLatestPublished(data);                                         \ // name##_ 表示实例化后的各个类的智能指针 比如 
                                                                                 // Chassis_ = unique_ptr(ChassisAdapter) = unique_ptr(Adapter<canbus::Chassis>) 
  }

/**
 * @class AdapterManager
 *
 * @brief this class hosts all the specific adapters and manages them.
 * It provides APIs for the users to initialize, access and interact
 * with the adapters that they are interested in.
 *
 * \par
 * Each (potentially) useful adapter needs to be registered here with
 * the macro REGISTER_ADAPTER.
 *
 * \par
 * The AdapterManager is a singleton.
 */
class AdapterManager {
 public:
  /**
   * @brief Initialize the /class AdapterManager singleton with the
   * provided configuration. The configuration is specified by the
   * file path.
   * @param adapter_config_filename the path to the proto file that
   * contains the adapter manager configuration.
   */
  static void Init(const std::string &adapter_config_filename);

  /**
   * @brief Initialize the /class AdapterManager singleton with the
   * provided configuration.
   * @param configs the adapter manager configuration proto.
   */
  static void Init(const AdapterManagerConfig &configs);

  /**
   * @brief check if the AdapterManager is initialized
   */
  static bool Initialized();

  static void Observe();

  /**
   * @brief create a timer which will call a callback at the specified
   * rate. It takes a class member function, and a bare pointer to the
   * object to call the method on.
   */
  template <class T>
  static ros::Timer CreateTimer(ros::Duration period,
                                void (T::*callback)(const ros::TimerEvent &),
                                T *obj, bool oneshot = false,
                                bool autostart = true) {
    CHECK(instance()->node_handle_)
        << "ROS node is only available in ROS mode, "
           "check your adapter config file!";
    return instance()->node_handle_->createTimer(period, callback, obj, oneshot,
                                                 autostart);
  }

 private:
  /// The node handler of ROS, owned by the /class AdapterManager
  /// singleton.
  std::unique_ptr<ros::NodeHandle> node_handle_;

  /// Observe() callbacks that will be used to to call the Observe()
  /// of enabled adapters.
  std::vector<std::function<void()>> observers_;

  bool initialized_ = false;

  /// The following code registered all the adapters of interest.
  REGISTER_ADAPTER(Chassis);
  REGISTER_ADAPTER(ChassisDetail);
  REGISTER_ADAPTER(ControlCommand);
  REGISTER_ADAPTER(Gps);
  REGISTER_ADAPTER(Imu);
  REGISTER_ADAPTER(Camera);
  REGISTER_ADAPTER(Localization);
  REGISTER_ADAPTER(Monitor);
  REGISTER_ADAPTER(Pad);
  REGISTER_ADAPTER(PerceptionObstacles);
  REGISTER_ADAPTER(Planning);
  REGISTER_ADAPTER(PointCloud);
  REGISTER_ADAPTER(Prediction);
  REGISTER_ADAPTER(TrafficLightDetection);
  REGISTER_ADAPTER(RoutingRequest);
  REGISTER_ADAPTER(RoutingResponse);
  REGISTER_ADAPTER(RelativeOdometry);
  REGISTER_ADAPTER(InsStat);
  REGISTER_ADAPTER(HMICommand);

  DECLARE_SINGLETON(AdapterManager);
};

}  // namespace adapter
}  // namespace common
}  // namespace apollo

#endif

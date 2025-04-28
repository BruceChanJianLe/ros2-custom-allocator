// ROS2
#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <rclcpp/allocator/allocator_common.hpp>
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include <pmrspy/pmrspy.hpp>

#include <cstdlib>
#include <memory>
#include <memory_resource>
#include <chrono>
#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/subscription_options.hpp>

int main (int argc, char *argv[])
{
  fmt::print(
    "This simple demo shows off a custom memory allocator to monitor all\n"
    "instances of memory allocation and deallocation in the program.\n"
    "It can be run in either regular mode (no argument) which is using\n"
    "unsynchronized_pool_resource or use monotonic_buffer_resource mode\n"
    "(by passing 'mono' as a command-line argument).  It will then publish a message to the\n"
    "'/allocator_tutorial' topic every 10 milliseconds until Ctrl-C is pressed.\n"
    "At that time it will print the number of published messages and received messages.\n"
    "All allocations and deallocations that happened during the program will be printed along the way.\n\n");

  bool mono{false};
  if (argc > 1)
  {
    if (std::string(argv[1]) == "mono")
    {
        mono = true;
    }
  }

  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  using Alloc = std::pmr::polymorphic_allocator<void>;
  using MsgType = std_msgs::msg::UInt32;

  pmrspy::print_alloc default_alloc{"Rogue Allocation!", std::pmr::null_memory_resource()};
  std::pmr::set_default_resource(&default_alloc);

  pmrspy::print_alloc oom{"Out of Memory", std::pmr::null_memory_resource()};

  // If used memory is more that 32kb, throw a std::bad_alloc
  std::array<std::uint8_t, 32768> buffer{};
  std::pmr::monotonic_buffer_resource
    underlying_bytes(buffer.data(), buffer.size(), &oom);

  pmrspy::print_alloc monotonic{"Monotonic Array", &underlying_bytes};

  std::pmr::unsynchronized_pool_resource unsync_pool(&monotonic);
  pmrspy::print_alloc pool("Pool", &unsync_pool);

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;

  // Let's use intra process by default
  auto context = rclcpp::contexts::get_global_default_context();
  auto options = rclcpp::NodeOptions()
    .context(context)
    .use_intra_process_comms(true);
  node = rclcpp::Node::make_shared("pmr_simple_node", options);

  std::shared_ptr<Alloc> alloc;
  // Create our resource as shared pointers (ros2 requirement)
  if (mono)
  {
    alloc = std::make_shared<Alloc>(&monotonic);
  }
  else
  {
    alloc = std::make_shared<Alloc>(&pool);
  }

  // Update publisher option
  rclcpp::PublisherOptionsWithAllocator<Alloc> pub_opts;
  pub_opts.allocator = alloc;

  // Create publisher
  auto pub = node->create_publisher<MsgType>(
      "allocator_tutorial", 10, pub_opts);

  // Update subscriber option
  rclcpp::SubscriptionOptionsWithAllocator<Alloc> sub_opts;
  sub_opts.allocator = alloc;

  // Subscriber requires memory strategy
  auto msg_mem_strat =
    std::make_shared<
      rclcpp::message_memory_strategy::MessageMemoryStrategy<MsgType, Alloc>>(alloc);

  // Create subsriber
  uint_fast32_t sub_counter{};
  auto sub = node->create_subscription<MsgType>("allocator_tutorial", 10, 
    [&sub_counter](MsgType::ConstSharedPtr msg) -> void
    {
      (void)msg;
      ++sub_counter;
    },
    sub_opts, msg_mem_strat
  );

  /*********************************************************/

  // Create a MemoryStrategy, which handles the allocations made by the Executor during the
  // execution path, and inject the MemoryStrategy into the Executor.
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<Alloc>>(alloc);

  rclcpp::ExecutorOptions exec_opts;
  exec_opts.memory_strategy = memory_strategy;
  rclcpp::executors::SingleThreadedExecutor exec(exec_opts);

  // Add our node to the executor
  exec.add_node(node);

  /*********************************************************/

  // You must also instantiate a custom deleter and allocator for use when allocating messages
  using MsgAllocTraits = rclcpp::allocator::AllocRebind<MsgType, Alloc>;
  using MsgAlloc = MsgAllocTraits::allocator_type;
  using MsgDeleter = rclcpp::allocator::Deleter<MsgAlloc, MsgType>;

  MsgDeleter msg_deleter;
  MsgAlloc msg_alloc = *alloc;
  rclcpp::allocator::set_allocator_for_deleter(&msg_deleter, &msg_alloc);

  /*********************************************************/

  rclcpp::sleep_for(std::chrono::milliseconds(1));

  using MsgUniquePtr = std::unique_ptr<MsgType, MsgDeleter>;

  uint_fast32_t pub_counter{};
  while (rclcpp::ok()) {
    // Create a message with the custom allocator, so that when the Executor deallocates the
    // message on the execution path, it will use the custom deallocate.
    auto ptr = MsgAllocTraits::allocate(msg_alloc, 1);
    MsgAllocTraits::construct(msg_alloc, ptr);
    MsgUniquePtr msg(ptr, msg_deleter);
    msg->data = pub_counter;
    ++pub_counter;
    pub->publish(std::move(msg));
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    exec.spin_some();
  }

  fmt::print("\n\nPublished {} msg(s)!\n\n", pub_counter);
  fmt::print("\n\nReceived {} msg(s)!\n\n", sub_counter);

  return EXIT_SUCCESS;
}

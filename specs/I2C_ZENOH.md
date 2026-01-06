In ROS 2 Kilted, Zenoh is a Tier-1 middleware. Using a **Zenoh Queryable** for your I2C bus allows you to bypass the heavier ROS 2 Service stack for raw hardware access while keeping your sensor nodes modular.

A "Queryable" is essentially a callback that is triggered only when someone asks for data. It’s perfect for I2C because it naturally serializes requests: if two nodes query the bus at the same time, Zenoh handles the queuing, and your bridge executes them one by one.

### 1. The I2C Bridge Node (The "Server")

This node sits on the hardware. It declares a `queryable` on a key expression like `pico/i2c/bus1`.

```cpp
#include <zenoh.hxx>
#include <iostream>

using namespace zenoh;

int main() {
    auto session = Session::open(Config::create_default());
    
    // Key pattern that this node will "answer" for
    auto key_expr = KeyExpr("pico/i2c/bus1/**");

    // Declare the queryable (The responder)
    auto queryable = session.declare_queryable(key_expr, [](const Query& query) {
        // 1. Parse parameters (e.g., "addr=0x68&len=6")
        std::string params = query.get_parameters();
        
        // 2. Perform Physical I2C Operation
        // lock_bus();
        // data = read_i2c(params);
        // unlock_bus();

        // 3. Reply to the specific requester
        query.reply(query.get_keyexpr(), Bytes::serialize("raw_sensor_data_here"));
    });

    // Keep node alive
    std::cout << "I2C Bridge is running. Press enter to stop..." << std::endl;
    std::cin.get();
}

```

---

### 2. The Sensor Node (The "Client")

This node doesn't know about the I2C bus file descriptor; it just "gets" a URI.

```cpp
#include <zenoh.hxx>

void poll_sensor() {
    auto session = Session::open(Config::create_default());
    
    // Query the bus with parameters in the selector
    auto selector = KeyExpr("pico/i2c/bus1/sensorA?addr=0x68&len=6");

    session.get(selector, [](const Reply& reply) {
        if (reply.is_ok()) {
            auto sample = reply.get_ok();
            auto data = sample.get_payload().deserialize<std::string>();
            // Process sensor data...
        }
    });
}

```

---

### Why this is better in Kilted:

* **Zero-Copy:** In Kilted, Zenoh can use shared memory. When your sensor node asks for data, the bytes can be passed through memory buffers without being "packed and unpacked" like a standard ROS message.
* **Logical Separation:** Your sensor node logic remains identical whether the I2C bus is on the same Raspberry Pi or a different microcontroller connected via Ethernet.
* **Natural Queueing:** Since the Bridge node handles the `queryable` callback, it acts as a single-threaded bottleneck for the I2C bus, which is exactly what you want for hardware that doesn't support concurrent access.

### Next Step

To get this running on your Pi 4 with Ubuntu 24.04, you’ll need the `rmw_zenoh_cpp` package.

**Would you like the commands to install the Zenoh router and set up a basic Kilted workspace with these two nodes?**
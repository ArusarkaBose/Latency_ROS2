# Latency_ROS2
**Latency measurements for ROS**  

Measure local environment or server delays with the help of Latency tests.

1. **[Latency checker for String messages](https://github.com/ArusarkaBose/Latency_ROS2/blob/master/README.md#latency-checker-for-string-messages-)**
   * **[Server-Client Setup](https://github.com/ArusarkaBose/Latency_ROS2/blob/master/README.md#server-client-setup-)**
   
</br></br></br>

## Latency Checker for String Messages :
### Server-Client Setup :
Run **devel_space/devel_space/latencycheckserver.py** followed by **devel_space/devel_space/latencycheckclient.py**

---
#### Algorithm:
1.  Create a **server** node and a **client** node

2. The **client** node gets its `rostime` and sends it as a `request` to the **server** node  

3. The **server** node receives the `request` and returns the same as `response` to the **client** node without any change

4. The **client** node measures the difference between its `rostime` at the instant of receipt of the **server** `response` and the `rostime` returned as `response` by the **server** (which is equal to the `rostime` of the **client** at the instant of sending the `service request`)

---

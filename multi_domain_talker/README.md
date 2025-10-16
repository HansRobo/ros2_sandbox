# MultiDomainTalker

## 起動

```bash
ros2 run multi_domain_talker multi_domain_talker_node
```

## node1からの受け取り

```bash
$ ROS_DOMAIN_ID=0 ros2 topic echo /topic
data: Hello World from domain 
---
data: Hello World from domain 0
---
data: Hello World from domain 0
---
data: Hello World from domain 0
---
data: Hello World from domain 0
```

## node2からの受け取り

```bash
$ ROS_DOMAIN_ID=1 ros2 topic echo /topic
data: Hello World from domain 1
---
data: Hello World from domain 1
---
data: Hello World from domain 1
---
data: Hello World from domain 1
---
data: Hello World from domain 1
```




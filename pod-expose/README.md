# Ways of accessing pods in clusters from the outside

A cluster is isolated by default, which reduces the burden of security breaches. But there are different ways of exposing pods to the outside, which are discussed with examples down below:

1. `hostNetwork` -> pods can see network interfaces on the host machine where pods were started
2. `hostPort` -> cotainer is exposed to external network at `nodeIP:containerPort`
3. `NodePort` -> builds on top of ClusterIP and allows gets random or specified by user port from range of 30000-32767. All cluster nodes listen to that port and forward all traffic to corresponding Service.
4. `LoadBalancer` -> Works with supported cloud providers and with Metallb for On Premise clusters. In addition to opening NodePort, Kubernetes creates cloud load balancer that forwards traffic to NodeIP:Nodeport for that service. 
5. `Ingress` (Ingress + Ingress Controller) -> Ingress-controller is exposed by Nodeport or LoadBalancer service and works as L7 reverse-proxy/LB for the cluster Services.
6. `kubectl proxy` -> provides a secure connection between the cluster(API Server) and the client

Inside the cluster, just to inlcude all the service types:

1. `ClusterIp` -> this is for making pods accessible inside cluster and across nodes, not from outside the cluster
2. `headless Service` (`ClusterIP: None`) -> if you want to interact with single specific pod endpoints directly instead of the ClusterIP 

#### Using `hostNetwork` -> pod level

This makes the Pod's services accessible through the node's IP address

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: nginx
spec:
  hostNetwork: true
  containers:
    - name: nginx
      image: nginx:alpine
```
Applications in pod can see all network interfaces of the host where it runs. It can listen on all interfaces and is also accessible on all interfaces.

Start the application:
```shell
kubectl apply -f hostNetwork.yml
```

Then you can check via GET request if application is running in two ways, both endpoints can be accessed with `kubectl describe node`:
1. With `kd node k3d-test-server-0 | grep InternalIP` you can get the IP address of the node
2. Or with `kubectl describe node k3d-test-server-0 | grep Hostname` you get the Hostname, in that case you need to make sure to configure local DNS service via /etc/hosts and add this static entry: `InternalIP` `Hostname` 

```shell
curl -v http://k3d-test-server-0:80
```

Not recommended because every time pod is restarted, it could be `rescheduled on different node` and there can be `port conflicts` because two applications running one same port can not run on the same node. 
Onepossible **Use case** for `hostNetwork: true` is when CNIs like Calico need full control of networking on every node.

#### Using `hostPort` -> container level


```yaml
apiVersion: v1
kind: Pod
metadata:
  name: influxdb
spec:
  containers:
    - name: influxdb
      image: influxdb
      ports:
        - containerPort: 8086
          hostPort: 8086
```

Start the application:
```shell
kubectl apply -f hostPort.yml
```

Curl it again:
```sh
curl -v http://k3d-test-server-0:8086/ping
```

Same drawbacks as `hostNetwork` approach. One possible **use case** is allowing hostPorts 80 and 443 to allow traffic on these ports from outside of the cluster.


#### Using `NodePort` -> service level

Instead of the Kubernetes internal service type `ClusterIP`, the service type `NodePort` can make it accessible for a port from the range 30000-32767, and each Kubernetes node will proxy that port to the pods selected by the service. So you do not have a classical service object, but rather the node as a proxy before the pods which is implemented via the kube-proxy, same as with ClusterIp and headless services. 

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: influxdb
  labels:
    name: influxdb
spec:
  containers:
    - name: influxdb
      image: influxdb
      ports:
        - containerPort: 8086
---
kind: Service
apiVersion: v1
metadata:
  name: influxdb-service
spec:
  type: NodePort
  ports:
    - port: 8086
      nodePort: 30000
  selector:
    name: influxdb
```

Start the application:
```shell
kubectl apply -f NodePort.yml
```

Curl it again:
```sh
curl -v http://k3d-test-server-0:30000/ping
```

The NodePort service represents a static endpoint through which the selected pods can be reached. Now for the case that there is a lot of traffic on different nodes and nodes can become unavailable, it makes sense to implement a LoadBalancer:

#### Using `LoadBalancer` -> service level

A cloud provider has to be enabled in the configuration of the Kubernetes cluster which provides built in cloud load balancers because it provides the endpoint to the cluster. In k3d in a local setup we can not test this, other than on cluster setup setting no loadbalancer with `--no-lb` and setting up or own load balancer. Otherwise per default you work with the loadbalancer of k3d: `Traefik` which does do port fowardubg for cluster and can be checked with `kubectl get services -n kube-system`. 

An example of LoadBalancer creation with nginx pods and an Ingress rule can be found in top level folder `load-balancing`. 

#### Apiserver as proxy 

The proxy provides a secure connection between the cluster(API Server) and the client, this avoid you having to change all your applications to implement a security logic just to communicate to the cluster, this way, you authenticate once, and every application use this secure connection without any changes.

Can be tested with two Terminals:

```shell
kubectl proxy --port=8080
```

```shell
curl http://localhost:8080/api/
```

With output:

```json
{
  "kind": "APIVersions",
  "versions": [
    "v1"
  ],
  "serverAddressByClientCIDRs": [
    {
      "clientCIDR": "0.0.0.0/0",
      "serverAddress": "10.0.2.15:8443"
    }
  ]
}
```


---

Source: inspired by this [article](https://alesnosek.com/blog/2017/02/14/accessing-kubernetes-pods-from-outside-of-the-cluster/) by Ales Nosek.

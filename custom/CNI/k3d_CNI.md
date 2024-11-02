# K3d architecture customization

To customize the architecture, we need to access the resources in the `kube-system` namespace, which are objects created by the Kubernetes System like `kube-dns`, `kube-proxy`, `metrics`, etc. and plugins like the CNI plugin. `kube-system` contains service accounts which are used to run the kubernetes controllers. These service accounts are granted significant permissions like creating pods anywhere. 

## K3D CNI customization 

Check out for more specific details and visualization of k3s architecture: [K3s-architecture](https://docs.k3s.io/architecture)

K3d is a wrapper around k3s and k3d runs k3s as docker images with the docker init system.
The use of k3d allows for tools like docker networking, but we can also customize the networking interface.

One server and one node setup:
```shell
kubectl get nodes
NAME                       STATUS   ROLES                  AGE   VERSION
k3d-k3s-default-agent-0    Ready    <none>                 35m   v1.30.4+k3s1
k3d-k3s-default-server-0   Ready    control-plane,master   35m   v1.30.4+k3s1
```

```sh
ps aux | grep k3s
# It's using Docker's init system to launch the K3d entrypoint script
# ENTRYPOINT keyword in docker is reserved to indicate the command the container is going to run as soon as it is initiated
/sbin/docker-init -- /bin/k3d-entrypoint.sh server --tls-san 0.0.0.0 --tls-san k3d-k3s-default-serverlb
# actual entrypoint script running, which sets up the environment for K3s
bin/sh /bin/k3d-entrypoint.sh server ...
# main K3s server process. It's responsible for running the control plane components of your Kubernetes cluster
/bin/k3s server
# agent process, which runs on each node
/bin/k3s agent
# Multiple Containerd processes are managing pods and containers. Containerd allows for low level container runtime interfaces.
/bin/containerd-shim-runc-v2 ...containerd.sock
```

While K3d uses Docker networks under the hood, inside the K3s containers, K3s is still managing its own networking, via a slightly customized Flannel implementation.
But how can we access k3s if we are using k3d?

## How to customize k3s inside k3d?

[K3d-Commands](https://k3d.io/v5.1.0/usage/commands/) show us that you can use `--k3s-arg`  which adds additional arguments to the k3s server/agent.
This way we can acces k3s containers while using k3d.

K3d also has a [guide](https://k3d.io/v5.7.4/usage/advanced/calico/) on how to implement Calico instead of Flannel.

Download Calico Manifest and save it with original file name:
```shell
curl https://k3d.io/v5.4.6/usage/advanced/calico.yaml -O
```

Wthout IP forwarding, no connection between pod-to-pod and node-to-node possible.
Modify calico.yaml, find the calico-config ConfgMap and add the `allow_ip_forwarding` setting in the `cni_network_config.plugins` section:
```yaml
        ...
      "plugins": [
        {
          "type": "calico",
          "container_settings": {
            "allow_ip_forwarding": true
          },
          ...
```

Create k3d/k3s cluster without Flannel and instead Calico for single node:
Most CNI plugins come with their own network policy engine, so it is recommended to set --disable-network-policy as well to avoid conflicts.
```shell
# docker needs full path of calico.yaml
k3d cluster create test --k3s-arg '--flannel-backend=none@server:*'   --k3s-arg '--disable-network-policy@server:*'   --volume "$(pwd)/calico.yaml:/var/lib/rancher/k3s/server/manifests/calico.yaml"
```

It works if you check `kubectl get pods -n kube-system` and you see the calico pods running.

Calico in k3d becomes problematic when it is multi node and you add more agent and server nodes with the `-a 2 -s 2` flags for example.

You need to properly expose the BGP Port 179 and configure calico.yaml further.

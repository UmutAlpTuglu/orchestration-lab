# Istio with K3d

**First some theory:**
- Disabling `Traefik` is necessary, so `Istio` can manage all traffic alone. `Traefik` handles incoming traffic only while Istio handles all traffic (incoming, outgoing and internal).
- Traefik has a single ingress point and Istio is a `Mesh of proxies`, where there is always one proxy next to each service -> Mesh in Networking is linking multiple access points, or "nodes" together.
  - in Istio there are `sidecar proxies (inudstry standard: envoy) running alongside services` in k8s and can manage L3 and L7 protocols, but can also do way more
    - like on L7 in HTTPS for example, they manage SSL termination, request authentication, retry logic, timeouts, tracing, circuit braking
      - could be implemented in application code but better to seperate it
    - the proxies can also manage security, like service-to-service encryption, access policies and identity based auth
- Service Meshes have a ton of different use cases
- Istio architecture consists of `Control plane` and `Data Plane`
  - Data Plane: network/mesh of sidecar/Envoy proxies
  - Control Plane:
    - Galley: Istios configuration management
    - Pilot: receives config from Galley, converts it to Envoy configurations and distributes it to each of envoy proxies in network
    - Citadel: provides each of the services with strong identities and generates certificates so services can trust each other and communicate via TLS


Create cluster without traefik:
```shell
k3d cluster create my-multinode-cluster --servers 1 --agents 3 --port 9080:80@loadbalancer --port 9443:443@loadbalancer --api-port 6443 --k3s-arg="--disable=traefik@server:0"
```

After that install [istioctl](https://istio.io/latest/docs/setup/additional-setup/download-istio-release/).

In Istio there are different `profiles` which can customize the Istio control plane and the side cars in data plane.
```shell
istioctl profile list
```

Some notes about the default profile:
- Ingress Gateway is enabled
- Egress Gateway is disabled
- Istiod is enabled

Istio only auto-injects the sidecar proxy into pods where there are namespaces labeled with istio-injection=enabled
```shell
istioctl install --set profile=default
kubectl label namespace default istio-injection=enabled
```

Apply all files in the folder:
```shell
kubectl apply -f service-meshes/
```

Istiod is the control plane, istio ingressgateway is the entry point(Envoy Proxy) for the cluster after traffic comes from k3d-proxy:
```shell
kgp -n istio-system
istio-ingressgateway-794797dbb8-sdwk8   1/1     Running   0          22m
istiod-6c854cfc5d-9fp84                 1/1     Running   0          22m
```

Traffic flow:
```text
External HTTP Request (curl http://127.0.0.1:9080)
↓
k3d-proxy (Docker Container)
↓
istio-ingressgateway (Istio Component and entry point)
↓
Gateway Resource (Defines allowed traffic)
↓
VirtualService (Routes traffic inside mesh in general and to services)
↓
Echo Server Service
↓
Pod [Echo Server + Envoy Sidecar]
```

Test if it worked:
```shell
curl http://127.0.0.1:9080
```

---

Inspiration: [k3d istio guide](https://brettmostert.medium.com/k3d-kubernetes-istio-service-mesh-286a7ba3a64f)

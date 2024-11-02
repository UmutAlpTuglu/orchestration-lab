# Load Balancing in k3d

In the folder `pod-expose` there are multiple ways of exposing pods. 
In this example it is about making ngingx pods accessible for http requests *from the server but also from the Internet*.

In k3d `Traefik` functions as both the `ingress controller` and `load balancer` of our k3d cluster.
- As an ingress controller: It interprets Ingress resources and sets up routing rules
- As a load balancer: It handles incoming traffic distribution and acts as the entry point for external traffic


**How to access from the internet:**

1. HTTP request to your-hostname.example.com hits UFW firewall of server -> Port 80 is configured and allowed for your IP address
2. Request reaches Traefik through k3d's port mapping (80:80@loadbalancer)
3. Traefik checks the Host header against Ingress rules and routes to ClusterIP Service
4. ClusterIP Service distributes traffic between the Nginx pods
5. After that HTTP response gets sent back 

## Implementation

Start the application with built in Traefik load balancer and forward all incoming http(80) to the k3d cluster running on server:
```shell
k3d cluster create my-cluster -p "80:80@loadbalancer" 
```

Get your external cluster IP via Traefik:
```shell
# Check for external IP of Traefik
kg services -n kube-system
```

Add new entry in /etc/hosts on server:
```shell
echo "<EXTERNAL_IP> your-hostname.example.com" | sudo tee -a /etc/hosts
```

This is just a basic nginx setup to test if you can access pods via curl on port 80. 


> The Ingress needs to point to the ClusterIP service.

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: nginx-deployment
spec:
  replicas: 2
  selector:
    matchLabels:
      app: nginx
  template:
    metadata:
      labels:
        app: nginx
    spec:
      containers:
        - name: nginx
          image: nginx:alpine
          ports:
            - containerPort: 80
---
apiVersion: v1
kind: Service
metadata:
  name: nginx-loadbalancer
spec:
  type: ClusterIP 
  ports:
    - port: 80
      targetPort: 80
  selector:
    app: nginx
---
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: nginx-ingress
  annotations:
    traefik.ingress.kubernetes.io/router.entrypoints: web
spec:
  ingressClassName: traefik 
  rules:
    - host: your-hostname.example.com
      http:
        paths:
          - path: /
            pathType: Prefix
            backend:
              service:
                name: nginx-loadbalancer
                port:
                  number: 80
```

Apply the three Kubernetes Resources above:
```shell
kubectl apply -f LoadBalancer.yml
```

Check if the resources were correctly created:
```shell
kubectl get pods,svc,ingress
```

After this you can GET welcome HTML file via curling the host we added:
```sh
curl -v http://your-hostname.example.com
#<!DOCTYPE html>
#<html>
#<head>
#<title>Welcome to nginx!</title>
#<style>
#...
```

Now if you want to host a web server just for your home server in k3d, you can deny all incoming traffic via ufw, allow ssh from your IP (for accessing server via SSH) and allow http requests to test the accessibility of your nginx pods via the internet:

```shell
ufw allow from YOUR_HOME_IPV4 to any port 80
ufw allow from YOUR_HOME_IPV6 to any port 80
```

And now you can send a GET request like this and receive it on your host. 
```shell
curl -v -H "Host: your-hostname.example.com" http://SERVER_PUBLIC_IP
#<!DOCTYPE html>
#<html>
#<head>
#<title>Welcome to nginx!</title>
#<style>
#...
```

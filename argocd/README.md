# ArgoCD in k3d

```shell
k3d cluster create argocd-cluster --agents 2
kubectl create namespace argocd
kubectl apply -n argocd -f https://raw.githubusercontent.com/argoproj/argo-cd/stable/manifests/install.yaml
kubectl wait --for=condition=Ready pods --all -n argocd --timeout=300s
```

Connect to UI when you are running cluster in remote server, via local SSH port forwarding:

```shell
ssh -L 8080:localhost:8080 USER@SERVER_IP
# On the server
kubectl port-forward svc/argocd-server -n argocd 8080:443
# on your host
localhost:8080 
# security warning just ignore, after that set password up
```

Admin password:
```shell
kubectl -n argocd get secret argocd-initial-admin-secret -o jsonpath="{.data.password}" | base64 -d; echo
```

[Install argocd cli](https://argo-cd.readthedocs.io/en/stable/cli_installation/)

Login in cli and change admin passwd
```shell
argocd login localhost:8080 --username admin --insecure
argocd account update-password
```

Deploy test app:
```shell
# Create a sample application
argocd app create guestbook \
  --repo https://github.com/argoproj/argocd-example-apps.git \
  --path guestbook \
  --dest-server https://kubernetes.default.svc \
  --dest-namespace default

# Sync the application
argocd app sync guestbook

# Verify deployment
kubectl get pods -n default
```

Play and test in UI and with CLI:
```shell
argocd app get guestbook
argocd app list
```

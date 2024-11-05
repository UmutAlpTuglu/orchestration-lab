# orchestration-lab

Playground for orchestration related services especially in a k3d/k3s environment.

## Prerequisites

If not available already, install the following:

- [Docker](https://docs.docker.com/engine/install/ubuntu/) 
- [K3D](https://k3d.io/v5.6.0/#installation)
- [kubectl](https://kubernetes.io/docs/tasks/tools/#kubectl)

## General Structure

- [pod-expose](pod-expose) tests all the ways to expose your cluster to the outside in k3d, except load balancing and ingress rules which are done in [load-balancing in k3d](load-balancing)
- [load-balancing in k3d](load-balancing) just tests how Traefik works in k3d and how you can use it to host a minimal web setup to allow traffic inside your cluster.
- just testing customization in Kubernetes, because big fan of customizing in Linux (Check out [nix-dotfiles](https://github.com/UmutAlpTuglu/nix-dotfiles)), by changing a crucial part of the k3s architecture. K2s uses Flannel CNI which runs faster than Calico, which is normally run in k8s archietctures. In [custom CNI folder](custom/CNI) I just remove the old CNI and add a new one in a minimal way.
- [service-meshes in k3d](service-meshes)
- app structure and pipelines are described below.

## Microservices Structure

The idea is to develop python apps inside microservices folder. The [py.dockerfile](py.dockerfile) can be used via `Dev Containers` Extensions in `vscode`, where you can find exemplary a nasa application. All new python dependencies (libs) can be added via [requirements.txt](requirements.txt). 

### Nasa App

**Prerequisites**

- create docker hub with read - write access token
- create Nasa API key
- have a server to deploy to
- create secrets for actions as mentioned in the pipeline (also add Nasa API key)

Here you can find the [details](microservices/nasa/README.md) of the app.
For the [nasa app](microservices/nasa) there is a [CI/CD pipeline](.github/workflows/nasa_deploy.yml) deployed where images get sent to a private docker hub registry and then are pulled to a server and the docker container is automatically run in detached mode on a server and saves the image in a designated folder (path can be altered in pipeline). 

The special thing about this pipeline is that it gets triggered via git tags, which also get used in all the subsequent jobs.
This is how to trigger it from your host:
```shell
git tag v1.0.1
git push origin v1.0.1
```

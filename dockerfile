# docker build . -f dockerfile -t alphafactorstrategy
FROM python:latest

RUN pip install --upgrade pip


RUN apt-get update && apt-get install --no-install-recommends  -y \
    git \
    openssh-client\
    && rm -rf /var/lib/apt/lists/* 

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt
RUN curl -sSL https://install.python-poetry.org | python3 -

RUN useradd -ms /bin/bash vscode
USER vscode
WORKDIR /workspace

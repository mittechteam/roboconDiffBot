# Docker Installation

This document describes how to install Docker.

## Step 1: Install Docker

Install Docker by following the instructions at [https://docs.docker.com/install/](https://docs.docker.com/install/).

## Step 2: Install Docker Compose

Install Docker Compose by following the instructions at [https://docs.docker.com/compose/install/](https://docs.docker.com/compose/install/).

## Step 3: Create a Personal Access Token (PAT)

Github Docs: [Creating a personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token#creating-a-personal-access-token-classic)

> Note: You must have the `read:packages` scope selected.

## Step 4: Authenticate with a PAT (Personal Access Token)

Github Docs: [Authenticating with a personal access token (classic)](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-with-a-personal-access-token-classic)

```bash
$ export CR_PAT=YOUR_TOKEN
```

```bash
$ echo $CR_PAT | docker login ghcr.io -u USERNAME --password-stdin

> Login Succeeded
```
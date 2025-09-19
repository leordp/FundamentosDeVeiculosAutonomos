# Makefile for managing the Docker containers

IMAGE = veiculosautonomos-coppeliasim
SHELL := bash
NAME := coppeliasim_container

.PHONY: build up down clean help

.PHONY: help
help: ## Show usage information for this Makefile.
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.PHONY: build
build:  ## build the Docker image
	docker compose build

.PHONY: shell
shell: ## Open a shell in the Docker container
	docker exec -it ${NAME} bash

.PHONY: run
run: ## Run the Docker container
	docker compose up --remove-orphans

.PHONY: run-detached
run-detached: ## Run the Docker container (detached mode)
	docker compose up -d --remove-orphans

.PHONY: dev
dev: ## Run the Docker container but don't load CoppeliaSim
	docker compose -f docker-compose.yml -f docker-compose.dev.yml up -d --remove-orphans && docker exec -it ${NAME} bash

.PHONY: stop
stop: ## Stop the Docker container
	docker kill ${NAME} || true

.PHONY: clean
clean: ## Clean up Docker containers and images
	docker compose down --volumes --remove-orphans
	docker system prune -f --volumes

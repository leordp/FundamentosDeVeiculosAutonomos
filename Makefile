# Makefile para gerenciar containers Docker do projeto

IMAGE ?= veiculosautonomos-coppeliasim
NAME  ?= coppeliasim_container
SHELL := bash

.PHONY: help build up up-d bash stop down clean logs

## -------------------------------------------------------------------
## 🔹 Comandos principais
## -------------------------------------------------------------------

help: ## Mostrar ajuda
	@echo -e "\nComandos disponíveis:\n"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	| awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'
	@echo ""

build: ## Construir a imagem Docker
	@echo "[INFO] Buildando imagem $(IMAGE)..."
	docker compose build

up: ## Subir container em background (detached)
	@echo "[INFO] Subindo container $(NAME) em background..."
	docker compose up -d --remove-orphans

shell: ## Entrar no container com bash
	@echo "[INFO] Entrando no container $(NAME)..."
	docker exec -it $(NAME) bash

stop: ## Parar o container
	@echo "[INFO] Parando container $(NAME)..."
	docker stop $(NAME) || true

down: ## Derrubar containers e redes
	@echo "[INFO] Derrubando stack Docker..."
	docker compose down --remove-orphans

logs: ## Ver logs do container
	docker compose logs -f $(NAME)

remove: ## Limpar containers, redes e volumes
	@echo "[INFO] Limpando container..."
	docker rm $(NAME)

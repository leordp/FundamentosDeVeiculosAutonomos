# Fundamentos de Veículos Autônomos

Repositório de apoio ao curso/projeto "Fundamentos de Veículos Autônomos".
Contém código para simulação e controle de um veículo (simulador) e módulos para uso em veículo real.

## Estrutura do repositório

- `docker-compose.yml` - configuração do ambiente Docker para rodar o CoppeliaSim e o workspace.
- `Makefile` - comandos úteis para construir, subir e gerenciar o container Docker.
- `fva_ws/` - workspace principal do projeto
  - `dockerfiles/Dockerfile` - Dockerfile usado pela composição
  - `src/fundamentos_veiculos_autonomos/` - código-fonte
    - `simulador/` - código do simulador (classe do carro, scripts de execução, recursos de pista)
    - `veiculo_real/` - módulos para integração com hardware real (sensores, servos, filtros, encoders, câmera)
    - `solution_stack/` - solução desenvolvida, scripts e dados captados
  - `startup/` - scripts e configurações para inicialização do ambiente

## Clonar o repositório (com submódulos)

Se você for clonar este repositório com submódulos, use os comandos abaixo antes de seguir para a etapa do Docker/Makefile:

```bash
# clonar o repositório principal
git clone https://github.com/leordp/FundamentosDeVeiculosAutonomos.git
cd FundamentosDeVeiculosAutonomos

# inicializar e atualizar submódulos (se houver)
git submodule update --init --recursive
```

Se preferir clonar já inicializando os submódulos de uma vez:

```bash
git clone --recurse-submodules https://github.com/leordp/FundamentosDeVeiculosAutonomos.git
```


## Como usar

Pré-requisitos:
- Docker e Docker Compose instalados
- X11 disponível se quiser usar interface gráfica (GUI) do simulador

Passos rápidos (modo comum com Makefile):

1. Build da imagem:

```bash
make build
```

2. Subir o container em background:

```bash
make up
```

3. Entrar no container (bash):

```bash
make shell
```

4. Parar e derrubar a stack:

```bash
make stop
make down
```

Observação: o `docker-compose.yml` mapeia o display X11 para permitir uso de GUI. Se estiver rodando localmente com X11, exporte a variável `DISPLAY` e permita acesso via `xhost +local:root`.

## Pasta de inicialização (startup)

A pasta `startup/` está incluída no workspace e contém scripts e configurações úteis para preparar o ambiente de desenvolvimento dentro do container ou numa sessão local. Itens relevantes encontrados:

- `start.sh` - script de inicialização (pode executar serviços, carregar variáveis ou iniciar sessões)
- `session.yml` - configuração de sessão (usada por ferramentas de inicialização)

Uso típico: dentro do container (ou localmente), você pode executar o script de startup para aplicar configurações e iniciar utilitários:

```bash
# dar permissão e executar (se necessário)
chmod +x fva_ws/startup/start.sh
./fva_ws/startup/start.sh
```

### Comandos básicos de tmux

O projeto inclui arquivos de configuração do `tmux` para facilitar o desenvolvimento em janelas/telas separadas. Aqui estão comandos básicos úteis:

- Listar sessões existentes:

```bash
tmux ls
```

- Anexar a uma sessão existente (por exemplo `fva`):

```bash
tmux attach -t fva
```

- Desanexar da sessão (dentro do `tmux`):

Pressione: `Ctrl+b` então `d`

- Criar janelas e dividir painéis:

Dentro do `tmux` (prefixo padrão `Ctrl+b`):

  - Nova janela: `c`
  - Alternar janelas: `n` (próxima), `p` (anterior)
  - Dividir verticalmente: `%`
  - Dividir horizontalmente: `"`

- Mover entre painéis (prefixo `Ctrl+b`):

  - `o` para alternar, ou use as teclas direcionais (`Arrow keys` com prefixo)

- Fechar janela/painel: `exit` ou `Ctrl+d` dentro do shell do painel

---


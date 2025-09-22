#!/bin/bash
set -e

# Caminho absoluto do script
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

# Nome do arquivo .ttt passado como argumento
if [ -z "$1" ]; then
    echo "Uso: ./start.sh <arquivo.ttt>"
    echo "Disponíveis: simulador_rampa.ttt simulador_cone.ttt"
    exit 1
fi

# Exporta variável para o tmuxinator usar
export SIM_FILE="/home/fva_ws/src/fundamentos_veiculos_autonomos/simulador/auxiliares/$1"

# Remove link antigo e recria
rm -f .tmuxinator.yml
ln -s session.yml .tmuxinator.yml

# Inicia tmuxinator com o arquivo .ttt selecionado
tmuxinator start -p session.yml

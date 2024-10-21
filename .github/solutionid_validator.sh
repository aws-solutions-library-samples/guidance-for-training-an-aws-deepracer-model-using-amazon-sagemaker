#!/bin/sh 
#set -e 

echo "checking solution id $1"
echo "grep -nr --exclude-dir='.github' "$1" ./.."
result=$(grep -nr --exclude-dir='.github' "$1" ./..)
if [ $? -eq 0 ]
then
  echo "Solution ID $1 found\n"
  echo "$result"
  exit 0
else
  echo "Solution ID $1 not found"
  exit 1
fi

export result

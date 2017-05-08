#!/bin/bash

if [ -f .bash_profile ]; then
  echo "You have a .bash_profile"
else
  echo "Yikes! you have no .bash_profile"
fi

if [ "$(id -u)" != "0" ]; then
  echo "You must be the superuser to run this script" >&2
  exit 1
else
 # home_space 
fi

function home_space
{
  if [ "$(id -u)" = "0" ]; then
    echo "Bytes Directory"
    du -s /home/* | sort -nr
    echo "pre"
  fi
}

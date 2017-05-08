#!/bin/bash

# echo -n "Enter some text >"
# read text #(default variable:REPLY)
# echo "You entered $text"

echo -n "Hurry up and type something > "
if read -t 3 responce; then
  echo "greatm you made in time $responce"
else
  echo "sorry, you miss it"
fi

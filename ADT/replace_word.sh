#!/bin/bash

# Replace all occurrences of a word with another word in .h files

find . -maxdepth 1 -type f -name "*.h" -print0 | while read -d $'\0' file
do
  # Replace all occurrences of "old_word" with "new_word" in the file
  sed -i 's/int/__host__ __device__\nint/g' "$file"
done

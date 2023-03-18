echo -n 44030F147A65 | perl -pe 's/([0-9a-f]{2})/chr hex $1/gie' > /dev/tcp/localhost/8889

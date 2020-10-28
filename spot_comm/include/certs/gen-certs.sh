#!/bin/bash

# Generate valid CA
openssl genrsa -passout pass:1234 -des3 -out ca.key 4096
openssl req -passin pass:1234 -new -sha512 -x509 -days 3650 -key ca.key -out ca.crt -subj  "/C=US/O=Boston Dynamics/CN=Boston Dynamics Root CA"

# Generate valid Server Key/Cert
openssl genrsa -passout pass:1234 -des3 -out server.key 4096
openssl req -passin pass:1234 -new -sha512 -key server.key -out server.csr -config gen-certs.conf
openssl x509 -req -passin pass:1234 -days 3650 -in server.csr -CA ca.crt -CAkey ca.key -set_serial 01 -extensions subjectAltName -out server.crt -extfile gen-certs.conf  -extensions v3_req

# Remove passphrase from the Server Key
openssl rsa -passin pass:1234 -in server.key -out server.key

# Generate valid Client Key/Cert
openssl genrsa -passout pass:1234 -des3 -out client.key 4096
openssl req -passin pass:1234 -new -sha512 -key client.key -out client.csr -subj  "/C=US/O=Boston Dynamics/CN=localhost"
openssl x509 -passin pass:1234 -req -days 3650 -in client.csr -CA ca.crt -CAkey ca.key -set_serial 01 -out client.crt

# Remove passphrase from Client Key
openssl rsa -passin pass:1234 -in client.key -out client.key

# Need to move ca.crt into bosdyn/client/resources/ as ca.pem (or symlink) and also need to reinstall bosdyn-client everytime

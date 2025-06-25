# Generate RSA keys properly (run this once to create your keys)
from Crypto.PublicKey import RSA

# Generate 2048-bit RSA key pair
key = RSA.generate(2048)

# Save private key
with open("server_private.pem", "wb") as f:
    f.write(key.export_key())

# Save public key in PKCS#1 format
with open("server_public.pem", "wb") as f:
    f.write(key.publickey().export_key())

print("RSA keys generated successfully")

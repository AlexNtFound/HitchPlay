# API server for ROAR
This repo contains the API server for the ROAR project.

## Quick Start
```bash
# create a virtual environment
conda create -n roar python=3.7
conda activate roar
```

```bash
# install dependencies
pip install -r requirements.txt
```

```bash
# run the server
python main.py
```

Go to `http://localhost:8000/docs` to see the API documentation.
Try out the API by invoking the following on a separate client
```bash
curl -X 'POST' \
  'http://SERVER_IP_ADDRESS:8000/v1/dev' \
  -H 'accept: application/json' \
  -H 'Content-Type: application/json' \
  -d '{
  "command": "echo hello world"
}'
```

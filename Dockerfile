FROM continuumio/miniconda3:latest

WORKDIR /app

# Ensure conda is in PATH
ENV PATH="/opt/conda/bin:$PATH"

# Install everything via conda
RUN conda install -y -c conda-forge python=3.11 pythonocc-core=7.7.0 fastapi uvicorn python-multipart

# Verify OCC at build time
RUN python -c "import OCC; print('OCC import OK')"

COPY main.py .

EXPOSE 8000

# Use python -m to ensure correct Python is used
CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]

FROM continuumio/miniconda3:latest

WORKDIR /app

# Install pythonocc-core via conda + Python deps
RUN conda install -y -c conda-forge python=3.11 pythonocc-core=7.7.0 && \
    pip install fastapi uvicorn python-multipart

# Fail build early if OCC not importable
RUN python -c "import OCC; print('OCC import OK')"

COPY main.py .

EXPOSE 8000

CMD ["conda", "run", "--no-capture-output", "-n", "base", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]

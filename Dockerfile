# Dockerfile for Railway deployment with FreeCAD + pythonOCC
# Updated for SolidWorks file support (SLDPRT/SLDASM → STEP conversion)

FROM continuumio/miniconda3:latest

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/conda/bin:$PATH"

# Install system dependencies (including FreeCAD)
RUN apt-get update && apt-get install -y --no-install-recommends \
    freecad \
    freecad-python3 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libfontconfig1 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create working directory
WORKDIR /app

# Create conda environment with pythonOCC
RUN conda create -n occ-env python=3.10 -y

# Install pythonOCC and FastAPI dependencies
RUN /bin/bash -c "source activate occ-env && \
    conda install -c conda-forge pythonocc-core=7.7.0 -y && \
    pip install fastapi uvicorn python-multipart pydantic"

# Copy application code
COPY main.py /app/

# Expose port
EXPOSE 8000

# Set the entrypoint to use conda environment
ENV PATH="/opt/conda/envs/occ-env/bin:$PATH"
ENV CONDA_DEFAULT_ENV=occ-env

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8000/health || exit 1

# Run the application
CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]

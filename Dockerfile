FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ODY_ROOT=/odyssey

RUN apt-get update && apt-get install -y \
    build-essential \
    make \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /odyssey

COPY . .

RUN make clean && make sim

ENTRYPOINT ["./sim"]
CMD ["dive240.mi"]

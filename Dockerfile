FROM golang:1.14.0-alpine3.11

#dependency for github.com/cpmech/gosl
WORKDIR /tmp
RUN apk add --no-cache alpine-sdk cmake && \
    wget -O metis.tar.gz http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/metis-5.1.0.tar.gz && \
    tar -xvf metis.tar.gz && \
    cd metis-5.1.0 && \
    make config && make install
    # apk del alpine-sdk cmake && \
    # rm -rf /tmp/*

WORKDIR /sort

ADD go.mod /sort
RUN go mod download

ADD / /sort
RUN go test

WORKDIR /sort/example
RUN go build -o /usr/bin/sort-example

CMD [ "sort-example" ]

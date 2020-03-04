FROM golang:1.14.0-alpine3.11

#dependency for github.com/cpmech/gosl
WORKDIR /tmp
RUN apk add --no-cache alpine-sdk cmake && \
    wget -O metis.tar.gz http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/metis-5.1.0.tar.gz && \
    tar -xvf metis.tar.gz && \
    cd metis-5.1.0 && \
    make config && make install && \
    apk del alpine-sdk cmake && \
    rm -rf /tmp/*

RUN mkdir /cam-event-detector
WORKDIR /cam-event-detector

ADD go.mod .
RUN go mod download

#now build source code
ADD . ./
RUN go build -o /usr/bin/cam-event-detector

ADD startup.sh /

# CMD [ "/startup.sh" ]

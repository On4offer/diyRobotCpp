FROM ubuntu:24.04 AS build
RUN apt-get update && apt-get install -y --no-install-recommends cmake g++ make ca-certificates && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY CMakeLists.txt ./
COPY include include
COPY src src
COPY apps apps
COPY tests tests
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DDIYROBOT_BUILD_QT=OFF -DDIYROBOT_BUILD_OPENCV=OFF \
 && cmake --build build --parallel \
 && ctest --test-dir build --output-on-failure

FROM ubuntu:24.04
COPY --from=build /src/build/diyrobot_cli /usr/local/bin/diyrobot_cli
ENTRYPOINT ["diyrobot_cli"]
CMD ["--help"]

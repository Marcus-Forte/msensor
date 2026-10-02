# The runtime image. 
# To build for arm64 (e.g for Raspberry pi, Nvidia Jetson, ...), simply pass `--platform linux/arm64` to `docker build`.

FROM mdnf1992/cpp-dev AS build

COPY . /src

WORKDIR /src

# Install the Release build and its runtime dependencies into `/app`.
RUN cmake --preset gcc -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/app \
    -DBUILD_TESTING=OFF && \
    cmake --build --preset build-gcc --config Release --target install --parallel 6

FROM debian:trixie-slim AS runtime

ENV GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0:/usr/local/lib/gstreamer-1.0

# gcc-16 needs libstdc++6 >= 16 (GLIBCXX_3.4.35), which trixie lacks: install it from sid.
RUN printf 'Types: deb\nURIs: http://deb.debian.org/debian\nSuites: sid\nComponents: main\nSigned-By: /usr/share/keyrings/debian-archive-keyring.pgp\n' > /etc/apt/sources.list.d/sid.sources && \
    apt-get update && apt-get install -y \
    libstdc++6 \
    libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 libgnutls30 libdw1 libunwind8 libi2c0 libjpeg62-turbo libgomp1 libegl1 libgles2 \
    libboost-log1.83.0 libboost-thread1.83.0 libboost-system1.83.0 libboost-iostreams1.83.0 libboost-atomic1.83.0 libboost-serialization1.83.0 \
    libflann1.9 liborc-0.4-0 libjsoncpp26 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /app /usr/local/
COPY --from=build /usr/local/share /usr/local/share/
COPY --from=build /usr/local/lib/aarch64-linux-gnu/ /usr/local/lib/aarch64-linux-gnu/
COPY --from=build --chmod=0755 /src/deployment/entrypoint.sh /usr/local/bin/entrypoint.sh

RUN ldconfig

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD []
# One can use `scratch` as a base and manually call the copied `ld` binary.
# RUN chmod +x /app/lib/ld-linux-aarch64.so.1
# FROM scratch AS runtime
# CMD ["/usr/local/lib/ld-linux-aarch64.so.1", "--library-path=/usr/local/lib", "/usr/local/bin/sim_publisher"]
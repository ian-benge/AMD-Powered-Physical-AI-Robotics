# Build through phase1_build_candidate.py: BuildKit needs a local tag in FROM.
# The wrapper verifies that separate tag against the immutable baseline ID.
ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS builder
USER root
COPY gz-rendering-8.2.3.tar.gz /tmp/rendering.tar.gz
COPY patch_renderer.py /tmp/patch_renderer.py
SHELL ["/bin/bash", "-c"]
RUN echo "5bfe18d164414e098b2ba6ef81bf9391f7ff8f2c39b8a4fa9f8543f778e921c8  /tmp/rendering.tar.gz" | sha256sum -c - \
 && mkdir /tmp/rendering-src \
 && tar -xzf /tmp/rendering.tar.gz -C /tmp/rendering-src --strip-components=1 \
 && python3 /tmp/patch_renderer.py /tmp/rendering-src \
 && source /opt/ros/jazzy/setup.bash \
 && cmake -S /tmp/rendering-src -B /tmp/rendering-build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF \
      -DSKIP_ogre2=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/jazzy/opt/gz_rendering_vendor \
 && cmake --build /tmp/rendering-build --target gz-rendering8-ogre -j 2

ARG BASE_IMAGE
FROM ${BASE_IMAGE}
USER root
# Replace only the Ogre backend inside this candidate image. The original
# core, Ogre2 backend and resource paths remain those of the baseline image.
COPY --from=builder /tmp/rendering-build/lib/libgz-rendering8-ogre.so.8.2.3 /opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre.so.8.2.3
COPY --from=builder /tmp/rendering-build/lib/libgz-rendering8-ogre.so.8.2.3 /opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins/libgz-rendering8-ogre.so.8.2.3
LABEL rover.rendering.phase="1" rover.rendering.source="gz-rendering8_8.2.3" rover.rendering.patch="opaque-texel-depth-v4"

FROM mbari/avedac-mbarivision-readannotations

ENV APP_HOME /usr/local

RUN groupadd -r --gid 716 docker && \
    useradd -r --uid 1001 -g docker dcline-admin && \
    chgrp -R docker ${APP_HOME} && \
    chmod -R 755 ${APP_HOME} && \
    chmod a+rx ${APP_HOME}/bin/readAnnotations

USER dcline-admin

CMD ${APP_HOME}/bin/readAnnotations

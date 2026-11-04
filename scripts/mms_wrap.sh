#!/usr/bin/env bash
# mms wrapper: logs the full protocol (requests & responses) so runs can be
# inspected/debugged afterwards without screenshots.
cd /home/lolo/micromouse_algo
export MM_PROTO=1
stdbuf -oL ./build/mms "$@" 2>>/tmp/opencode/algo_err.log | tee -a /tmp/opencode/algo_out.log

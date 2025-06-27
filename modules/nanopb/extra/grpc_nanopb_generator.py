#!/usr/bin/env python3
"""
Generate gRPC service boilerplate code for Zephyr HTTP/2 server integration.

Usage:
    grpc_nanopb_generator.py <proto_file> <out_dir>
Generates <out_dir>/<proto_basename>.pb.grpc.h
"""

import sys
import os
import subprocess
import tempfile
from google.protobuf import descriptor_pb2


def generate_service_code(fdesc):
    code = []
    code.append('/* Automatically generated gRPC service boilerplate */')
    code.append('#pragma once')
    # Include nanopb generated header
    code.append('#include "%s.pb.h"' % os.path.splitext(os.path.basename(fdesc.name))[0])
    code.append('#include <zephyr/net/http/service.h>')
    code.append('#include <zephyr/net/http/server.h>')
    code.append('')
    code.append('#ifdef __cplusplus')
    code.append('extern "C" {')
    code.append('#endif')
    code.append('')
    pkg = fdesc.package
    for service in fdesc.service:
        service_name = service.name
        full_service = service_name if not pkg else pkg + '.' + service_name
        svc_ident = service_name.lower()
        for method in service.method:
            method_name = method.name
            path = '/%s/%s' % (full_service, method_name)
            m_ident = '%s_%s' % (svc_ident, method_name.lower())
            code.append('/* gRPC service %s.%s */' % (service_name, method_name))
            code.append('int %s(struct http_client_ctx *client, enum http_data_status status, const struct http_request_ctx *request_ctx, struct http_response_ctx *response_ctx, void *user_data);' % m_ident)
            code.append('static struct http_resource_detail_dynamic %s_detail = {' % m_ident)
            code.append('    .common = { BIT(HTTP_METHOD_POST), HTTP_RESOURCE_TYPE_DYNAMIC, %d, NULL, "application/grpc" },' % len(path))
            code.append('    .cb = %s,' % m_ident)
            code.append('    .holder = NULL,')
            code.append('    .user_data = NULL,')
            code.append('};')
            code.append('HTTP_RESOURCE_DEFINE(%s, %s, "%s", &%s_detail);' % (m_ident, svc_ident, path, m_ident))
            code.append('')
    code.append('#ifdef __cplusplus')
    code.append('}')
    code.append('#endif')
    code.append('')
    return '\n'.join(code)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    proto_file = sys.argv[1]
    out_dir = sys.argv[2]
    # Generate descriptor set for the .proto file
    fd, desc_file = tempfile.mkstemp()
    os.close(fd)
    cmd = ['protoc', '--include_imports', '--descriptor_set_out=%s' % desc_file, proto_file]
    if subprocess.call(cmd) != 0:
        sys.exit(1)
    fds = descriptor_pb2.FileDescriptorSet()
    with open(desc_file, 'rb') as f:
        fds.ParseFromString(f.read())
    os.remove(desc_file)
    # Locate the file descriptor
    fdesc = None
    for f in fds.file:
        if os.path.basename(f.name) == os.path.basename(proto_file):
            fdesc = f
            break
    if fdesc is None:
        return
    if not fdesc.service:
        # No services to generate
        return
    # Generate code and write to .pb.grpc.h
    code = generate_service_code(fdesc)
    proto_basename = os.path.splitext(os.path.basename(proto_file))[0]
    out_file = os.path.join(out_dir, proto_basename + '.pb.grpc.h')
    with open(out_file, 'w') as f:
        f.write(code)
    print('Generated gRPC boilerplate: %s' % out_file)


if __name__ == '__main__':
    main()
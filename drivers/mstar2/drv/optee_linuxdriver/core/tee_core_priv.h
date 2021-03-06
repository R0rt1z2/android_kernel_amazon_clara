/*
 * Copyright (c) 2014, STMicroelectronics International N.V.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef __TEE_CORE_PRIV_H__
#define __TEE_CORE_PRIV_H__

#include "linux/tee_core.h"
#include "linux/tee_ioc.h"

#ifndef CONFIG_TEE_2_4
struct tee_ioctl_param {
        __u64 attr;
        __u64 a;
        __u64 b;
        __u64 c;
};

struct tee_param_memref {
        size_t shm_offs;
        size_t size;
        struct tee_shm *shm;
};

struct tee_param_value {
        u64 a;
        u64 b;
        u64 c;
};

struct tee_ioctl_version_data {
        __u32 impl_id;
        __u32 impl_caps;
        __u32 gen_caps;
};

 struct tee_param {
        u64 attr;
        union {
                struct tee_param_memref memref;
                struct tee_param_value value;
        } u;
};

#define TEE_IOCTL_UUID_LEN		16
 struct tee_ioctl_open_session_arg {
        __u8 uuid[TEE_IOCTL_UUID_LEN];
        __u8 clnt_uuid[TEE_IOCTL_UUID_LEN];
        __u32 clnt_login;
        __u32 cancel_id;
        __u32 session;
        __u32 ret;
        __u32 ret_origin;
        __u32 num_params;
        /* num_params tells the actual number of element in params */
        struct tee_ioctl_param params[];
};

struct tee_ioctl_invoke_arg {
        __u32 func;
        __u32 session;
        __u32 cancel_id;
        __u32 ret;
        __u32 ret_origin;
        __u32 num_params;
        /* num_params tells the actual number of element in params */
        struct tee_ioctl_param params[];
};
#endif

/* from tee_core_module.c */
int tee_get(struct tee *tee);
int tee_put(struct tee *tee);

void tee_inc_stats(struct tee_stats_entry *entry);
void tee_dec_stats(struct tee_stats_entry *entry);

/* from tee_context.c */
int tee_context_dump(struct tee *tee, char *buff, size_t len);

struct tee_context *tee_context_create(struct tee *tee);
void tee_context_destroy(struct tee_context *ctx);

void tee_context_get(struct tee_context *ctx);
void tee_context_put(struct tee_context *ctx);

struct tee_shm *tee_context_create_tmpref_buffer(struct tee_context *ctx,
						 size_t size,
						 const void *buffer, int type);
struct tee_shm *tee_context_alloc_shm_tmp(struct tee_context *ctx, size_t size,
					  const void *data, int type);
int tee_context_copy_from_client(const struct tee_context *ctx, void *dest,
				 const void *src, size_t size);

/* from tee_session.c */
int tee_session_create_fd(struct tee_context *ctx, struct tee_cmd_io *cmd_io);
struct tee_session *tee_session_create_and_open(struct tee_context *ctx,
						struct tee_cmd_io *cmd_io);
int tee_session_close_and_destroy(struct tee_session *sess);

struct tee *tee_get_tee(const char *devname);
int tee_session_invoke_be(struct tee_session *sess, struct tee_cmd_io *cmd_io);

#endif

import os
import libsql_experimental as libsql


def connect_db():
    url = os.getenv("TURSO_DATABASE_URL")
    auth_token = os.getenv("TURSO_AUTH_TOKEN")
    print(auth_token)
    print(url)
    conn = libsql.connect("ceciadb.db", sync_url=url, auth_token=auth_token)
    conn.sync()
    return conn

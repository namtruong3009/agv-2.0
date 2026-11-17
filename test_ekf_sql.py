
import os
import pyodbc

server = os.environ.get("AGV_SQL_HOST", "172.31.32.1")
database = os.environ.get("AGV_SQL_DB", "AGV_Config")
username = os.environ.get("AGV_SQL_USER", "agv_runtime")
password = os.environ["AGV_SQL_PASSWORD"]

conn_str = (
    "DRIVER={ODBC Driver 18 for SQL Server};"
    f"SERVER={server},1433;"
    f"DATABASE={database};"
    f"UID={username};"
    f"PWD={password};"
    "Encrypt=yes;"
    "TrustServerCertificate=yes;"
)

with pyodbc.connect(conn_str, timeout=5) as conn:
    cursor = conn.cursor()

    cursor.execute("""
        SELECT ParameterName, ParameterValue
        FROM dbo.EKF_Parameters
        ORDER BY ParameterName
    """)

    for name, value in cursor.fetchall():
        print(f"{name} = {value}")

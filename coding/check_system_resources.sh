# filename: check_system_resources.sh
#!/bin/bash

echo "正在获取系统资源使用情况..."
echo "----------------------------------------"
echo "CPU 使用率最高的进程:"
top -b -n 1 | grep -v '%' | awk '{print $8, $9, $10, $11, $12}' | sort -nr | head -n 5

echo "----------------------------------------"
echo "内存使用最多的进程:"
ps aux | sort -k 4,4nr | head -n 5 | awk '{printf "%s\t%s\t%s\t%s\n", $1, $2, $3, $4}'
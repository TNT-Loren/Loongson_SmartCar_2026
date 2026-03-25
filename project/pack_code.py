import os

#  python3 pack_code.py


# 1. 你想打包的文件后缀名（过滤掉无关文件）
ALLOWED_EXTENSIONS = {'.c', '.cpp', '.h', '.hpp'}

# 2. 你想忽略的文件夹（极其重要！防止打包进几十MB的二进制垃圾或别人写的库）
IGNORE_DIRS = {'.git', 'build', 'out', 'libraries'} 

OUTPUT_FILE = 'project_code.txt'

def pack_code():
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as outfile:
        # os.walk 会从当前目录 '.' 开始向下级级遍历
        for root, dirs, files in os.walk('.'):
            # 在原地修改 dirs 列表，踢掉那些在 IGNORE_DIRS 里的文件夹
            dirs[:] = [d for d in dirs if d not in IGNORE_DIRS]
            
            for file in files:
                ext = os.path.splitext(file)[1] # 获取后缀名
                if ext in ALLOWED_EXTENSIONS:
                    filepath = os.path.join(root, file)
                    
                    # 写入显眼的文件分割线和相对路径
                    outfile.write(f"\n{'='*50}\n")
                    outfile.write(f"FILE: {filepath}\n")
                    outfile.write(f"{'='*50}\n\n")
                    
                    try:
                        # 强制使用 utf-8 读取，防止中文注释乱码报错
                        with open(filepath, 'r', encoding='utf-8') as infile:
                            outfile.write(infile.read())
                    except Exception as e:
                        # 如果遇到编码极其奇葩的文件，记录错误但程序不崩溃
                        outfile.write(f"// 读取文件失败: {e}\n")
                        
    print(f"打包完成！代码已保存至: {OUTPUT_FILE}")

if __name__ == '__main__':
    pack_code()
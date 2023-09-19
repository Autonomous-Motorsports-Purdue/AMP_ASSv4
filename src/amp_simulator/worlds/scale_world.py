import re

# Script to convert substring of the form `translation 1 2 3` into `translation 10 20 30`
# Needed for scale conversions in wbt world

f = open('Track2.wbt', "r")
target_str = f.read()

def repl(stri):
    arr = stri.group().split(' ')

    arr[1] = str(float(arr[1]) * 10)
    arr[2] = str(float(arr[2]) * 10)
    arr[3] = str(float(arr[3]) * 10)

    resu = ' '.join(arr)
    # print(resu)
    return resu

res_str = re.sub(r"scale\s-?(\d*\.?\d*)\s-?(\d*\.?\d*)\s-?(\d*\.?\d*)", repl, target_str)
print(res_str)

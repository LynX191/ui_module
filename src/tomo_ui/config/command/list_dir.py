import os
import getpass

username = getpass.getuser()

print('>>>>>')
try:
    print('CartonVision_Start')
    files = sorted(os.listdir(f"/home/{username}/tomo_stats/CartonVision"))
    for i in files:
        print(i)
    print('CartonVision_End')
except:
    print("List Folder Error")


try:
    print('StimCartoner_Start')
    files = sorted(os.listdir(f"/home/{username}/tomo_stats/StimCartoner"))
    for i in files:
        print(i)
    print('StimCartoner_End')
except:
    print("List Folder Error")

try:
    print('PimCartonCounting_Start')
    files = sorted(os.listdir(f"/home/{username}/tomo_stats/PimCartonCounting"))
    for i in files:
        print(i)
    print('PimCartonCounting_End')
except:
    print("List Folder Error")

try:
    print('exception_images_Start')
    files = sorted(os.listdir(f"/home/{username}/tomo_stats/exception_images"))
    for i in files:
        print(i)
    print('exception_images_End')
except:
    print("List Folder Error")

print('<<<<<')


import multiprocessing
import subprocess
import time
import os

# Your foo function
def foo(n):
    #os.system("roslaunch final_project main.launch")
    child = subprocess.Popen(["roslaunch","final_project","main.launch"])
    try:
        child.wait(100)
    except:
        child.kill()

if __name__ == '__main__':
    #os.system("roslaunch final_project main.launch")


    # Start foo as a process
    x = 10
    y = 10

    #TEST: Below is testing material
    roslaunch = subprocess.Popen(["roslaunch","final_project","main.launch"])
    try:
        roslaunch.wait(100)
    except:
        roslaunch.kill()



    time.sleep(5)
    Robot1 = subprocess.Popen(["rosrun", "final_project","robot1.py"])

    time.sleep(5)
    Robot1.kill()
    #roslaunch.kill()



    """
    p = multiprocessing.Process(target=foo, name="Foo", args=(10,))
    p.start()
    print("Thats a win")
    time.sleep(10)
    p.join(20)

    # If thread is active
    if p.is_alive():
        print("foo is running... let's kill it...")
        # Terminate foo
        #os.system("kill 3097")
        p.terminate()
        p.join()


    
    p = multiprocessing.Process(target=foo, name="Foo", args=(10,))
    p.start()

    p.join(20)

    # If thread is active
    if p.is_alive():
        print("foo is running... let's kill it...")

        # Terminate foo
        p.terminate()
        p.join()
        #finList.append(20)
    """
#TEST: Above is testing material


    """
    for i in range(11):
        for j in range(11):
            for k in range(11):
                for l in range(11):
                    for m in range(11):
                        for n in range(11):
                            xp = i/10
                            xi = j/10
                            xd = k/10
                            ap = l/10
                            ai = m/10
                            ad = n/10
                            p = multiprocessing.Process(target=foo, name="Foo", args=(x,y,xp, xi, xd, ap, ai, ad, finList))
                            p.start()

                            p.join(20)

                # If thread is active
                            if p.is_alive():
                                print("foo is running... let's kill it...")

                                # Terminate foo
                                p.terminate()
                                p.join()
                                finList.append(20)
                                """

import multiprocessing
import time
import os

# Your foo function
def foo(n):
    for i in range(10000 * n):
        print( "Tick")
        time.sleep(1)

if __name__ == '__main__':
    # Start foo as a process



    os.system("roslaunch final_project main.launch")
    print("DONE")
    x = 10
    y = 10

    #TEST: Below is testing material
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

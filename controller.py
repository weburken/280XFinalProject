import multiprocessing
import time
import robot1

# Your foo function
def foo(x,y,xp, xi, xd, ap, ai, ad):
    robot1.run(x,y,xp, xi, xd, ap, ai, ad)

if __name__ == '__main__':
    # Start foo as a process
    x = 10
    y = 10
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
                            p = multiprocessing.Process(target=foo, name="Foo", args=(x,y,xp, xi, xd, ap, ai, ad,))
                            p.start()

                            p.join(20)

                # If thread is active
                            if p.is_alive():
                                print("foo is running... let's kill it...")

                                # Terminate foo
                                p.terminate()
                                p.join()
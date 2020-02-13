# Simplifying trajectory math

Known Formula:

* *x = vx\* t*
* *y = vy \* t - g \* t² / 2*
* *vx = v0\* cos*
* *vy = v0\* sin*
* *t = x/vx*

>If we are trying to solve for v0 we need to first understand the formula for y:

* *y = vy \* x/vx - g \* (x/vx)² / 2*

>Because time is an unknown variable, we first replace t with the formula for time - x/vx in both instances. Next we want to isolate the *x* and put *vy* and *vx* together because they are like terms.

* *vy \* x / vy = x \* (vy / vx)*

>Now that we have *vy\*vx*, we can put in the value of vertical and horizontal velocity:

* *y = v0 \* sinθ / v0 * cos - g \* (x/vx)² / 2*


>And since *v0 / v0* is just 1- and *sin / cos* = tanθ we can simplify this further to get *x\*tanθ*.
>Next we need to ‘simplify’ *g \*(x/vx)² / 2*. We can replace vxwith *v0\* cos*.

* *y = xtanθ - g(x / v0 \* cosθ)² / 2*

>To simplify this further, we can multiply g into x.

* *g/1 \* (x / v0\*cosθ)² / 2 = gx² * v0² \* cos²θ / 2* 

>Finally, we can put 2 into the denominator so we don’t have to divide by 2- and we get the final trajectory formula of:

* *y = xtanθ - (gx²) / 2v0² \* cos²θ*

## Solving for Initial Velocity

>Now to solve for velocity, we need to manipulate the formula to solve for v. First let’s start to isolate v by adding *gx²/2v0²\*cosθ²* to both sides and subtracting y.

* *gx² / 2v0² \* cos²θ = xtanθ - y*

>Next we can multiply both sides by multiplying each side by *2v0²\*cos²θ*.

* *gx² = (xtanθ - y) \* 2v0² \* cos²θ*

>Now is the funnest part! We get to divide the entire problem by everything that isn’t *v0²*!

* *gx² / (xtanθ - y) / cosθ² / 2= v0²*

>Wooh! Now we can just take the square root of the whole thing to get rid of that pesky 2 on v0², to get a final velocity formula of:

* *sqrt(gx²/xtanθ - y) /cosθ² = v0)*

For a `gcc` project to use `librandomx.a` on x86_64:
- include `randomx.h`;
- link `librandomx.a`, **and** its dependency `stdc++`;

Also note that:
- The api source code for `librandomx.a` can be found at [github](https://github.com/xiongyw/RandomX/tree/master/src).
- Those api source code are pretty much C code using minimum C++ "features".
- The `librandomx.a` in this folder was built via cmake (`cmake -DARCH=native ..`) on x86_64.



import control.matlab as matlab
import control
import matplotlib.pyplot as plt
import numpy as np
from sympy import *


# вар 17
# 1. ky = 21
# 2. koc = 0.3
# 3. Tг = 4 c
# 4. Ty = 4 c
# 5. Турбина Паро-
# 6. Tгт = -
# 7. Tпт = 6 c
# 8. kпт = 3
# 9. O.C. = Ж
# 10. Тос = -

ky = 21.
koc = 0.132275
Tg = 4.
Ty = 4.
Tpt = 6.
kpt = 3.

time = np.arange(0, 500, 0.01)
freq = np.arange(0, 10, 0.01)
Woc = control.tf([koc], [1.])  # Обратная связь
Wg = control.tf([1.], [Tg, 1.])  # Генератор
Wt = control.tf([kpt], [Tpt, 1.])  # Паровая турбина
Wy = control.tf([ky], [Ty, 1.])  # Исполнительное устройство
Wraz = Wg * Wt * Wy # Передаточная функция разомкнутой системы
W = matlab.feedback(Wraz, Woc) # Передаточная функция замкнутой системы
print(W)
# 1. Снять переходную характеристику
y, x = matlab.step(W, time)
plt.title("Переходная характеристика")
plt.grid()
plt.plot(x, y)
plt.show()
# 2. Определить значения полюсов передаточной функции замкнутой САУ
z = matlab.pole(W)
print(z)
matlab.pzmap(W)
plt.title("Значения полюсов")
plt.grid()
plt.plot()
plt.show()
# 3. Разомкнуть САУ и оценить устойчивость по критерию Найквиста. Определить запасы устойчивости по модулю и по фазе
matlab.nyquist(W, freq)
plt.title("Годограф Найквиста")
plt.grid()
plt.plot()
plt.show()
# 4. Построить годограф Михайлова
coefficient = np.array(control.tfdata(W)[1][0][0])
w = control.tf(coefficient, [1.])
x = [re(w(1j * omega)) for omega in range(1000)]
y = [im(w(1j * omega)) for omega in range(1000)]
plt.title("Годогроф Михайлова")
plt.grid()
plt.plot(x, y)
plt.show()
# 5. Снять логарифмическую амплитудную частотную и логарифмическую фазовую частотную характеристики разомкнутой системы
matlab.bode(W, dB=False)
plt.grid()
plt.plot()
plt.show()
# 6. На основании алгебраического критерия Рауса–Гурвица рассчитать предельное значение k_(ос пред),
# при котором САУ находиться на границе устойчивости
for koc in np.arange(0, 0.2, 0.000001):
    Woc = control.tf([koc], [1.])
    W = matlab.feedback(Wraz, Woc)
    coefficient = np.array(control.tfdata(W)[1][0][0])
    a = coefficient.reshape(2, 2)
    b = np.flip(a.T, 0)
    if (np.linalg.det(b) >= -0.001) & (np.linalg.det(b) <= 0.001):
        print(koc)
        break


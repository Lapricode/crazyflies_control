1) Έχω μερικές διαφάνειες "crazyflies_getting_started" με γενικά στοιχεία και αρχικές οδηγίες σχετικά με το crazyflie και το setup του. Περιγράφεται ακόμα με αναλυτικό τρόπο το δυναμικό μοντέλο του crazyflie.

2) Για το localization του crazyflie έχουμε το lighthouse positioning system. Περισσότερα γι' αυτό, την εγκατάστασή του και το calibration, μπορείτε να δείτε στους παρακάτω συνδέσμους:
https://www.bitcraze.io/documentation/system/positioning/ligthouse-positioning-system/
https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/
Το calibration γίνεται μέσω του cfclient ανοίγοντας το tab "Lighthouse Positioning", καλό θα είναι να κάνετε "Save system config" το configuration της αρένας μετά το πέρας του "geometry estimation".

3) Έχω δημιουργήσει γραφικές παραστάσεις στο geogebra (αρχείο "crazyflie_dynamics_curves.ggb") για τις συναρτησιακές σχέσεις μεταξύ διαφόρων χρήσιμων μεγεθών του δυναμικού μοντέλου του crazyflie. Π.χ. (όλες οι παρακάτω σχέσεις αναφέρονται σε 1 μόνο από τους 4 κινητήρες):
  - rotor_speed (rad/sec) vs. PWM:         	omegar = sqrt(8e-4 * PWM^2 + 53.33 * PWM)
  - PWM vs. rotor_speed (rad/sec):         	PWM = -33333.0 + sqrt(1250.0 * omegar^2 + 1111111111.0)
  - thrust (N) vs. rotor_speed (rad/sec):  	Fi = kf * omegar^2 = 2.25e-8 omegar^2
  - thrust (N) vs. PWM:                    	Fi = 1.8e-11 * PWM^2 + 1.2e-6 * PWM
  - PWM vs. normal. thrust (N):                 PWM = -33333.0 + sqrt(8663836225.0 * norm_Fi + 1111111111.0)
  - normalized PWM vs. normalized thrust (N):   norm_PWM = -0.50863 + sqrt(0.25871 + 2.01727 * norm_Fi)
Συμβουλεύτηκα τις ακόλουθες εργασίες:
https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8905295&fileOId=8905299 (Appendix A συγκεκριμένα)
https://www.research-collection.ethz.ch/handle/20.500.11850/214143 (Chapter 3 συγκεκριμένα)

4) Το original firmware μπορείτε να βρείτε στον σύνδεσμο https://github.com/bitcraze/crazyflie-firmware. Για να δημιουργήσετε τον δικό σας controller υπάρχουν αναλυτικές οδηγίες στον σύνδεσμο: https://www.bitcraze.io/2023/02/adding-an-estimator-or-controller/.
Εδώ καταγράφω επίσης μερικές από τις παρατηρήσεις μου (βρίσκονται και στο fork μου https://github.com/Lapricode/crazyflie-firmware):
- Στον φάκελο examples/my_controller_estimator/:
  - δημιουργούμε τo src/my_controller.c (εδώ έχω την απαραίτητη appMain, αναγκαίες οι συναρτήσεις ControllerOutOfTreeInit, ControllerOutOfTreeTest και ControllerOutOfTree)
  - γίνονται οι κατάλληλες αλλαγές στα app-config και src/Kbuild
- Στον φάκελο src/modules/src:
  - η συνάρτηση controlMotors του stabilizer.c καλεί την powerDistribution του αρχείου power_distribution_quadrotor.c, η οποία αναλόγως το επιλεγμένο controlMode καλεί την κατάλληλη συνάρτηση που δίνει τα σήματα PWM στα motors (έχω υλοποιήσει την powerDistributionForce βάσει της powerDistributionForceTorque).
- Στο αρχείο src/modules/src/controller/controller.c:
  - απενεργοποίησα τον forced controller σχολιάζοντας τις γραμμές "#elif defined(CONFIG_CONTROLLER_OOT) \ #define CONTROLLER ControllerTypeOot", προκειμένου να είναι εφικτή η αλλαγή του controller μέσω του cfclient ή του python api.

5) Για το Python API, μπορείτε να βρείτε χρήσιμα πράγματα στο repo μου https://github.com/Lapricode/crazyflies_control. Ειδικότερα:
  - speeds_LQR_crazyflie.py και thrusts_LQR_crazyflie.py για τον LQR controller μου με rotor speeds και με thrusts αντίστοιχα
  - visualize_crazyflie.py για την προσομοίωση με matplotlib
- Στον φάκελο crazyflies_control/crazyflies_python_api:
  - position_control.py έχει τον high level commander για ορισμό θέσης και yaw γωνίας, προφανώς με μερικές τροποποίησεις μπορεί να γίνει και παρακολούθηση τροχιάς. Ακόμα, φαίνεται ο τρόπος για να παρακολουθείτε συγκεκριμένα log variables και να θέτετε παραμέτρους, όπως τον stabilizer.controller (είναι ο δείκτης του array controllerFunctions στο αρχείο src/modules/src/controller/controller.c του firmware, οι παράμετροι ορίζονται και μέσω του cfclient, από το tab Parameters)
  - swarm_choreography.py περιέχει απλό προγραμματάκι για την εκτέλεση μιας κυκλικής τροχιάς από σμήνος 4 crazyflies
  - check_uris.py για την ανίχνευση των uris των crazyflies, μπορεί να γίνει και με σύνδεση usb με το crazyflie μέσω του cfclient και πηγαίνοντας στο Connect->Configure 2.x
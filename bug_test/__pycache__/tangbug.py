ó
}Oí]c           @   s&   d  d l  Z d e f d     YZ d S(   iÿÿÿÿNt
   TangentBugc           B   s   e  Z d    Z d   Z RS(   c         C   s   g  |  _  d |  _ | |  _ d S(   s6   
        Function:
            Initialization
        g333333Ó?N(   t   boundary_memoryt   continuity_thresholdt   scan_max(   t   selft   laser_maximum(    (    s5   /home/ao/catkin_ws/src/bugtest/src/scripts/tangbug.pyt   __init__   s    		c   
      C   sf  g  } g  } xSt  t |   D]?} | | } | | } t |  d k r¥ | |  j k  r | j | t j |  | t j |  g  | } | }	 q^d } d }	 q | | |  j k sÇ | |  j k rR| j | t j |	  | t j |	  g  | j t j	 |   g  } | |  j k  r^| j | t j |  | t j |  g  q^q | } | }	 q W| S(   sÙ   
        Function:
            Find out how many line segments there are for each obstacle.

        Args:
            ranges      ->      laser ray's ranges
            angles      ->      laser ray's angles
        i    N(
   t   ranget   lenR   t   appendt   npt   cost   sint   NoneR   t   array(
   R   t   rangest   anglest   all_objectst   current_objectt   it   current_ranget   current_anglet   previous_ranget   previous_angle(    (    s5   /home/ao/catkin_ws/src/bugtest/src/scripts/tangbug.pyt
   Continuity   s*    	

-		"-3
(   t   __name__t
   __module__R   R   (    (    (    s5   /home/ao/catkin_ws/src/bugtest/src/scripts/tangbug.pyR       s   		(   t   numpyR
   t   objectR    (    (    (    s5   /home/ao/catkin_ws/src/bugtest/src/scripts/tangbug.pyt   <module>   s   
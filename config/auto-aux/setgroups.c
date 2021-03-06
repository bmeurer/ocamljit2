/***********************************************************************/
/*                                                                     */
/*                           Objective Caml                            */
/*                                                                     */
/*  Copyright 2009 Institut National de Recherche en Informatique et   */
/*  en Automatique.  All rights reserved.  This file is distributed    */
/*  under the terms of the GNU Library General Public License, with    */
/*  the special exception on linking described in file ../../LICENSE.  */
/*                                                                     */
/***********************************************************************/

/* Contributed by Stephane Glondu <steph@glondu.net> */

/* $Id$ */

#include <errno.h>

#include <sys/types.h>
#include <limits.h>
#include <grp.h>

int main(void)
{
  gid_t gidset[1];
  gidset[0] = 0;
  if (setgroups(1, gidset) == -1 && errno != EPERM) return 1;
  return 0;
}

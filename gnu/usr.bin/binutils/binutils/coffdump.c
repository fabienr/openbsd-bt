/* Coff file dumper.
   Copyright 1994, 1995, 1998, 1999, 2000, 2001, 2002, 2003
   Free Software Foundation, Inc.

   This file is part of GNU Binutils.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or (at
   your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

/* Written by Steve Chamberlain <sac@cygnus.com>

   This module reads a type tree generated by coffgrok and prints
   it out so we can test the grokker.  */

#include "bfd.h"
#include "libiberty.h"

#include "coffgrok.h"
#include "bucomm.h"
#include "getopt.h"

static int atnl;

static void tab (int);
static void nl (void);
static void dump_coff_lines (struct coff_line *);
static void dump_coff_type (struct coff_type *);
static void dump_coff_where (struct coff_where *);
static void dump_coff_visible (struct coff_visible *);
extern void dump_coff_symbol (struct coff_symbol *);
static void dump_coff_scope (struct coff_scope *);
static void dump_coff_sfile (struct coff_sfile *);
static void dump_coff_section (struct coff_section *);
extern void coff_dump (struct coff_ofile *);
static void show_usage (FILE *, int);
extern int main (int, char **);

static void
tab (int x)
{
  static int indent;
  int i;

  if (atnl)
    {
      if (x < 0)
	{
	  printf (")");
	  indent += x;

	  return;
	}
      else
	{
	  printf ("\n");
	  atnl = 0;
	}
    }

  if (x == -1)
    {
      for (i = 0; i < indent; i++)
	printf ("   ");

      indent += x;
      printf (")");
      return;
    }

  indent += x;

  for (i = 0; i < indent; i++)
    printf ("   ");

  if (x)
    {
      printf ("(");
    }
}

static void
nl (void)
{
  atnl = 1;
}

static void
dump_coff_lines (struct coff_line *p)
{
  int i;
  int online = 0;

  tab (1);
  printf (_("#lines %d "),p->nlines);

  for (i = 0; i < p->nlines; i++)
    {
      printf ("(%d 0x%x)", p->lines[i], p->addresses[i]);

      online++;

      if (online > 6)
	{
	  nl ();
	  tab (0);
	  online = 0;
	}
    }
  nl ();
  tab (-1);
}

static void
dump_coff_type (struct coff_type *p)
{
  tab (1);
  printf ("size %d ", p->size);

  switch (p->type)
    {
    case coff_secdef_type:
      printf ("section definition at %x size %x\n",
	      p->u.asecdef.address,
	      p->u.asecdef.size);
      nl ();
      break;
    case coff_pointer_type:
      printf ("pointer to");
      nl ();
      dump_coff_type (p->u.pointer.points_to);
      break;
    case coff_array_type:
      printf ("array [%d] of", p->u.array.dim);
      nl ();
      dump_coff_type (p->u.array.array_of);
      break;
    case coff_function_type:
      printf ("function returning");
      nl ();
      dump_coff_type (p->u.function.function_returns);
      dump_coff_lines (p->u.function.lines);
      printf ("arguments");
      nl ();
      dump_coff_scope (p->u.function.parameters);
      tab (0);
      printf ("code");
      nl ();
      dump_coff_scope (p->u.function.code);
      tab(0);
      break;
    case coff_structdef_type:
      printf ("structure definition");
      nl ();
      dump_coff_scope (p->u.astructdef.elements);
      break;
    case coff_structref_type:
      if (!p->u.aenumref.ref)
	printf ("structure ref to UNKNOWN struct");
      else
	printf ("structure ref to %s", p->u.aenumref.ref->name);
      break;
    case coff_enumref_type:
      printf ("enum ref to %s", p->u.astructref.ref->name);
      break;
    case coff_enumdef_type:
      printf ("enum definition");
      nl ();
      dump_coff_scope (p->u.aenumdef.elements);
      break;
    case coff_basic_type:
      switch (p->u.basic)
	{
	case T_NULL:
	  printf ("NULL");
	  break;
	case T_VOID:
	  printf ("VOID");
	  break;
	case T_CHAR:
	  printf ("CHAR");
	  break;
	case T_SHORT:
	  printf ("SHORT");
	  break;
	case T_INT:
	  printf ("INT ");
	  break;
	case T_LONG:
	  printf ("LONG");
	  break;
	case T_FLOAT:
	  printf ("FLOAT");
	  break;
	case T_DOUBLE:
	  printf ("DOUBLE");
	  break;
	case T_STRUCT:
	  printf ("STRUCT");
	  break;
	case T_UNION:
	  printf ("UNION");
	  break;
	case T_ENUM:
	  printf ("ENUM");
	  break;
	case T_MOE:
	  printf ("MOE ");
	  break;
	case T_UCHAR:
	  printf ("UCHAR");
	  break;
	case T_USHORT:
	  printf ("USHORT");
	  break;
	case T_UINT:
	  printf ("UINT");
	  break;
	case T_ULONG:
	  printf ("ULONG");
	  break;
	case T_LNGDBL:
	  printf ("LNGDBL");
	  break;
	default:
	  abort ();
	}
    }
  nl ();
  tab (-1);
}

static void
dump_coff_where (struct coff_where *p)
{
  tab (1);
  switch (p->where)
    {
    case coff_where_stack:
      printf ("Stack offset %x", p->offset);
      break;
    case coff_where_memory:
      printf ("Memory section %s+%x", p->section->name, p->offset);
      break;
    case coff_where_register:
      printf ("Register %d", p->offset);
      break;
    case coff_where_member_of_struct:
      printf ("Struct Member offset %x", p->offset);
      break;
    case coff_where_member_of_enum:
      printf ("Enum Member offset %x", p->offset);
      break;
    case coff_where_unknown:
      printf ("Undefined symbol");
      break;
    case coff_where_strtag:
      printf ("STRTAG");
    case coff_where_entag:
      printf ("ENTAG");
      break;
    case coff_where_typedef:
      printf ("TYPEDEF");
      break;
    default:
      abort ();
    }
  nl ();
  tab (-1);
}

static void
dump_coff_visible (struct coff_visible *p)
{
  tab (1);
  switch (p->type)
    {
    case coff_vis_ext_def:
      printf ("coff_vis_ext_def");
      break;
    case coff_vis_ext_ref:
      printf ("coff_vis_ext_ref");
      break;
    case coff_vis_int_def:
      printf ("coff_vis_int_def");
      break;
    case coff_vis_common:
      printf ("coff_vis_common");
      break;
    case coff_vis_auto:
      printf ("coff_vis_auto");
      break;
    case coff_vis_autoparam:
      printf ("coff_vis_autoparam");
      break;
    case coff_vis_regparam:
      printf ("coff_vis_regparam");
      break;
    case coff_vis_register:
      printf ("coff_vis_register");
      break;
    case coff_vis_tag:
      printf ("coff_vis_tag");
      break;
    case coff_vis_member_of_struct:
      printf ("coff_vis_member_of_struct");
      break;
    case coff_vis_member_of_enum:
      printf ("coff_vis_member_of_enum");
      break;
    default:
      abort ();
    }
  nl ();
  tab (-1);
}

void
dump_coff_symbol (struct coff_symbol *p)
{
  tab (1);
  printf ("List of symbols");
  nl ();

  while (p)
    {
      tab (1);
      tab (1);
      printf ("Symbol  %s, tag %d, number %d", p->name, p->tag, p->number);
      nl ();
      tab (-1);
      tab (1);
      printf ("Type");
      nl ();
      dump_coff_type (p->type);
      tab (-1);
      tab (1);
      printf ("Where");
      dump_coff_where (p->where);
      tab (-1);
      tab (1);
      printf ("Visible");
      dump_coff_visible (p->visible);
      tab (-1);
      p = p->next;
      tab (-1);
    }
  tab (-1);
}

static void
dump_coff_scope (struct coff_scope *p)
{
  if (p)
    {
      tab (1);
      printf ("List of blocks %lx ",(unsigned long) p);

      if (p->sec)
	printf( "  %s %x..%x",  p->sec->name,p->offset, p->offset + p->size -1);

      nl ();
      tab (0);
      printf ("*****************");
      nl ();

      while (p)
	{
	  tab (0);
	  printf ("vars %d", p->nvars);
	  nl ();
	  dump_coff_symbol (p->vars_head);
	  printf ("blocks");
	  nl ();
	  dump_coff_scope (p->list_head);
	  nl ();
	  p = p->next;
	}

      tab (0);
      printf ("*****************");
      nl ();
      tab (-1);
    }
}

static void
dump_coff_sfile (struct coff_sfile *p)
{
  tab (1);
  printf ("List of source files");
  nl ();

  while (p)
    {
      tab (0);
      printf ("Source file %s", p->name);
      nl ();
      dump_coff_scope (p->scope);
      p = p->next;
    }
  tab (-1);
}

static void
dump_coff_section (struct coff_section *ptr)
{
  int i;

  tab (1);
  printf ("section %s %d %d address %x size %x number %d nrelocs %d",
	  ptr->name, ptr->code, ptr->data, ptr->address,ptr->size,
	  ptr->number, ptr->nrelocs);
  nl ();

  for (i = 0; i < ptr->nrelocs; i++)
    {
      tab (0);
      printf ("(%x %s %x)",
	      ptr->relocs[i].offset,
	      ptr->relocs[i].symbol->name,
	      ptr->relocs[i].addend);
      nl ();
    }

  tab (-1);
}

void
coff_dump (struct coff_ofile *ptr)
{
  int i;

  printf ("Coff dump");
  nl ();
  printf ("#souces %d", ptr->nsources);
  nl ();
  dump_coff_sfile (ptr->source_head);

  for (i = 0; i < ptr->nsections; i++)
    dump_coff_section (ptr->sections + i);
}

char * program_name;

static void
show_usage (FILE *file, int status)
{
  fprintf (file, _("Usage: %s [option(s)] in-file\n"), program_name);
  fprintf (file, _(" Print a human readable interpretation of a SYSROFF object file\n"));
  fprintf (file, _(" The options are:\n\
  -h --help              Display this information\n\
  -v --version           Display the program's version\n\
\n"));

  if (status == 0)
    fprintf (file, _("Report bugs to %s\n"), REPORT_BUGS_TO);

  exit (status);
}

int
main (int ac, char **av)
{
  bfd *abfd;
  struct coff_ofile *tree;
  char **matching;
  char *input_file = NULL;
  int opt;
  static struct option long_options[] =
    {
      { "help", no_argument, 0, 'h' },
      { "version", no_argument, 0, 'V' },
      { NULL, no_argument, 0, 0 }
    };

#if defined (HAVE_SETLOCALE) && defined (HAVE_LC_MESSAGES)
  setlocale (LC_MESSAGES, "");
#endif
#if defined (HAVE_SETLOCALE)
  setlocale (LC_CTYPE, "");
#endif
  bindtextdomain (PACKAGE, LOCALEDIR);
  textdomain (PACKAGE);

  program_name = av[0];
  xmalloc_set_program_name (program_name);

  expandargv (&ac, &av);

  while ((opt = getopt_long (ac, av, "HhVv", long_options,
			     (int *) NULL))
	 != EOF)
    {
      switch (opt)
	{
	case 'H':
	case 'h':
	  show_usage (stdout, 0);
	  break;
	case 'v':
	case 'V':
	  print_version ("coffdump");
	  exit (0);
	case 0:
	  break;
	default:
	  show_usage (stderr, 1);
	  break;
	}
    }

  if (optind < ac)
    {
      input_file = av[optind];
    }

  if (!input_file)
    fatal (_("no input file specified"));

  abfd = bfd_openr (input_file, 0);

  if (!abfd)
    bfd_fatal (input_file);

  if (! bfd_check_format_matches (abfd, bfd_object, &matching))
    {
      bfd_nonfatal (input_file);

      if (bfd_get_error () == bfd_error_file_ambiguously_recognized)
	{
	  list_matching_formats (matching);
	  free (matching);
	}
      exit (1);
    }

  tree = coff_grok (abfd);

  coff_dump (tree);
  printf ("\n");

  return 0;
}

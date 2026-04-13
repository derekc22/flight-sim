module slicot_care_wrapper_mod
    use iso_c_binding, only: c_double, c_int, c_ptr, c_f_pointer
    implicit none
contains

    subroutine control_solve_care_sb02md(n, a_ptr, g_ptr, q_ptr, x_ptr, rcond, info) bind(C, name="control_solve_care_sb02md")
        integer(c_int), value :: n
        type(c_ptr), value :: a_ptr, g_ptr, q_ptr, x_ptr
        real(c_double), intent(out) :: rcond
        integer(c_int), intent(out) :: info

        real(c_double), pointer :: a_in(:,:), g_in(:,:), q_in(:,:), x_out(:,:)
        real(c_double), allocatable :: a_work(:,:), g_work(:,:), q_work(:,:), s(:,:), u(:,:), wr(:), wi(:), dwork(:)
        integer(c_int), allocatable :: iwork(:)
        logical, allocatable :: bwork(:)
        integer(c_int) :: lda, ldg, ldq, lds, ldu, ldwork
        character(len=1) :: dico, hinv, uplo, scal, sort
        external :: sb02md

        call c_f_pointer(a_ptr, a_in, [n, n])
        call c_f_pointer(g_ptr, g_in, [n, n])
        call c_f_pointer(q_ptr, q_in, [n, n])
        call c_f_pointer(x_ptr, x_out, [n, n])

        lda = max(1_c_int, n)
        ldg = max(1_c_int, n)
        ldq = max(1_c_int, n)
        lds = max(1_c_int, 2_c_int * n)
        ldu = max(1_c_int, 2_c_int * n)
        ldwork = max(2_c_int, 6_c_int * n)

        allocate(a_work(lda, n), g_work(ldg, n), q_work(ldq, n), s(lds, 2 * n), u(ldu, 2 * n), wr(2 * n), wi(2 * n), dwork(ldwork), iwork(2 * n), bwork(2 * n))

        dico = 'C'
        hinv = 'D'
        uplo = 'U'
        scal = 'N'
        sort = 'S'

        a_work = a_in
        g_work = 0.5d0 * (g_in + transpose(g_in))
        q_work = 0.5d0 * (q_in + transpose(q_in))

        call sb02md(dico, hinv, uplo, scal, sort, n, a_work, lda, g_work, ldg, q_work, ldq, rcond, wr, wi, s, lds, u, ldu, iwork, dwork, ldwork, bwork, info)

        if (info == 0) then
            x_out = q_work
        else
            x_out = 0.0d0
        end if

        deallocate(a_work, g_work, q_work, s, u, wr, wi, dwork, iwork, bwork)
    end subroutine control_solve_care_sb02md

end module slicot_care_wrapper_mod
